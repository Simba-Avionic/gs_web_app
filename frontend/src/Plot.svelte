<script>
    // @ts-nocheck
    import { onMount, onDestroy } from "svelte";
    import Plotly from "plotly.js-basic-dist-min";
    import { fetchConfig, rosTimeToFormattedTime } from "./lib/Utils.svelte";
    import { theme } from "./js/theme.js";
    import { cssVar, lightThemeColors, darkThemeColors } from "./js/colors.js";
    import { getPlotLayout } from "./js/plot_config.js";

    export let field;
    export let onRemove;

    let time_range = 1;
    const storageKey = `plot_range_${field.topic}_${field.val_name}`;

    let color;
    let plotDiv;
    let latestValue = "--";
    let ws;
    let buffer = [];
    let flushInterval;

    let xData = [];
    let yData = [];

    $: $theme, updateTheme();

    // Helper to calculate the buffered Y range
    function getBufferedYRange(data) {
        if (field.type === "bool") return [-0.1, 1.1];
        if (data.length === 0) return [0, 100];

        const min = Math.min(...data);
        const max = Math.max(...data);
        const padding = (max - min) * 0.15; // 15% padding

        // If value is constant (min === max), add fixed padding so it's not a flat line at the edge
        if (padding === 0) return [min - 1, max + 1];

        return [min - padding, max + padding];
    }

    function updateTheme() {
        if (!plotDiv || !plotDiv.hasChildNodes()) return;
        const layout = getPlotLayout(field);
        const relayoutObj = {};

        if (layout.plot_bgcolor !== undefined)
            relayoutObj["plot_bgcolor"] = layout.plot_bgcolor;
        if (layout.paper_bgcolor !== undefined)
            relayoutObj["paper_bgcolor"] = layout.paper_bgcolor;
        if (layout.font?.color) relayoutObj["font.color"] = layout.font.color;

        if (layout.xaxis) {
            if (layout.xaxis.tickfont)
                relayoutObj["xaxis.tickfont"] = layout.xaxis.tickfont;
            if (layout.xaxis.gridcolor)
                relayoutObj["xaxis.gridcolor"] = layout.xaxis.gridcolor;
        }

        if (layout.yaxis) {
            if (layout.yaxis.tickfont)
                relayoutObj["yaxis.tickfont"] = layout.yaxis.tickfont;
            if (layout.yaxis.gridcolor)
                relayoutObj["yaxis.gridcolor"] = layout.yaxis.gridcolor;
        }

        if (Object.keys(relayoutObj).length === 0) return;
        Plotly.relayout(plotDiv, relayoutObj);
    }

    function getRandomColor() {
        const colors = $theme === "dark" ? darkThemeColors : lightThemeColors;
        return colors[Math.floor(Math.random() * colors.length)];
    }

    function setTimeRange(min) {
        time_range = min;
        localStorage.setItem(storageKey, min.toString());
        closeWebSocket();
        fetchData();
    }

    async function fetchData() {
        try {
            const res = await fetch(
                `http://${window.location.host}/${field.topic}/query?field_name=${field.parent ? `${field.parent}/${field.val_name}` : field.val_name}&time_range=${time_range}`,
            );
            const data = await res.json();

            let rawX = data.records.map((entry) =>
                new Date(entry._time).toISOString(),
            );
            let rawY = data.records.map((entry) => entry._value);

            if (field.type === "bool") rawY = rawY.map((v) => (v ? 1 : 0));
            latestValue = rawY.length > 0 ? rawY[rawY.length - 1] : "--";

            const newestMs = rawX.length
                ? Date.parse(rawX[rawX.length - 1])
                : Date.now();
            const windowStartMs = newestMs - time_range * 60 * 1000;

            const paired = rawX.map((x, i) => ({
                tMs: Date.parse(x),
                tIso: x,
                v: rawY[i],
            }));
            const filtered = paired.filter((p) => p.tMs >= windowStartMs);
            const finalPairs = filtered.length
                ? filtered
                : paired.slice(-Math.max(1, Math.floor(time_range * 60)));

            xData = finalPairs.map((p) => p.tIso);
            yData = finalPairs.map((p) => p.v);

            await Plotly.newPlot(
                plotDiv,
                [
                    {
                        x: xData,
                        y: yData,
                        type: "scatter",
                        mode: "lines",
                        line: { color: color, width: 1.5 },
                        name: field.val_name,
                        hoverinfo: "x+y",
                    },
                ],
                getPlotLayout(field),
                { responsive: true, displayModeBar: false },
            );

            if (xData.length) {
                Plotly.relayout(plotDiv, {
                    "xaxis.range": [xData[0], xData[xData.length - 1]],
                    "yaxis.range": getBufferedYRange(yData), // Dynamic initial scale
                });
            }

            openWebSocket();
        } catch (error) {
            console.error("Error fetching or plotting data:", error);
        }
    }

    function openWebSocket() {
        ws = new WebSocket(`ws://${window.location.host}/${field.topic}`);
        ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                let v = field.parent
                    ? msg[field.parent][field.val_name]
                    : msg[field.val_name];
                if (v === undefined) return;
                if (field.type === "bool") v = v ? 1 : 0;

                const t = rosTimeToFormattedTime(
                    true,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                );
                buffer.push({ t, v });
            } catch (err) {
                /* quiet */
            }
        };

        flushInterval = setInterval(() => {
            if (buffer.length === 0) return;

            const last = buffer[buffer.length - 1];
            buffer = [];

            xData.push(last.t);
            yData.push(last.v);
            latestValue = last.v;

            const lastMs = Date.parse(last.t);
            const windowStartMs = lastMs - time_range * 60 * 1000;

            // Trim arrays
            const cutoffMs = windowStartMs;
            while (xData.length > 0 && Date.parse(xData[0]) < cutoffMs) {
                xData.shift();
                yData.shift();
            }

            try {
                const update = {
                    "xaxis.range": [
                        new Date(windowStartMs).toISOString(),
                        last.t,
                    ],
                };

                if (field.type !== "bool") {
                    // Calculate buffer
                    const minV = Math.min(...yData);
                    const maxV = Math.max(...yData);
                    const pad = (maxV - minV) * 0.15 || 1;

                    update["yaxis.range"] = [minV - pad, maxV + pad];
                    // Ensure tickvals is null so Plotly auto-generates 1, 10, 50...
                    update["yaxis.tickvals"] = null;
                    update["yaxis.ticktext"] = null;
                }

                Plotly.relayout(plotDiv, update);
                Plotly.restyle(plotDiv, { x: [xData], y: [yData] }, [0]);
            } catch (err) {
                console.warn("Plotly update error:", err);
            }
        }, 500);
    }

    function closeWebSocket() {
        if (flushInterval) {
            clearInterval(flushInterval);
            flushInterval = null;
        }
        if (ws) {
            ws.onmessage = null;
            ws.onclose = null;
            ws.onerror = null;
            try {
                ws.close();
            } catch (err) {
                console.warn("Error closing WebSocket:", err);
            }
            ws = null;
        }
    }

    onMount(() => {
        const savedRange = localStorage.getItem(storageKey);
        if (savedRange) {
            time_range = parseInt(savedRange, 10);
        }

        color = field.color ?? getRandomColor();
        fetchData();
    });

    onDestroy(() => {
        closeWebSocket();
        Plotly.purge(plotDiv);
    });
</script>

<div class="plot-cell">
    <div class="plot-header">
        <div class="plot-controls">
            <button
                class:selected={time_range === 1}
                on:click={() => setTimeRange(1)}>1m</button
            >
            <button
                class:selected={time_range === 10}
                on:click={() => setTimeRange(10)}>10m</button
            >
            <button
                class:selected={time_range === 30}
                on:click={() => setTimeRange(30)}>30m</button
            >
            <button
                class:selected={time_range === 60}
                on:click={() => setTimeRange(60)}>1h</button
            >
        </div>
        <div class="plot-info">
            <h5>
                {field.msg_type} — {field.alt_name
                    ? field.alt_name
                    : field.val_name}
            </h5>
        </div>
        <div class="latest-value">
            {typeof latestValue === "number"
                ? latestValue.toFixed(1)
                : latestValue}
            {field.unit ? field.unit : ""}
        </div>
        <button
            class="remove-btn"
            on:click={() => {
                closeWebSocket(); // cleanup internal socket
                onRemove(); // tell parent to remove this plot
            }}
            aria-label="Remove plot"
        >
            ×
        </button>
    </div>

    <div class="plot" bind:this={plotDiv}></div>
</div>

<style>
    .plot-cell {
        position: relative;
        background: var(--snd-bg-color);
        border-radius: 6px;
        display: flex;
        flex-direction: column;
        justify-content: flex-start;
        align-items: stretch;
        overflow: hidden;
        height: 100%;
        min-height: 0;
    }

    .plot-header {
        display: flex;
        flex: 0 0 auto;
        align-items: center;
        justify-content: space-between;
        background-color: var(--snd-bg-color);
        color: var(--text-color);
        font-size: 0.85rem;
        padding: 8px 8px;
    }

    .plot-info h5 {
        margin: 0;
        font-size: 0.85rem;
    }

    .plot-controls {
        display: flex;
        gap: 4px;
    }

    .plot-controls button {
        /* padding: 2px 6px; */
        font-size: 0.8rem;
        background-color: var(--bg-color);
        color: var(--text-color);
        border: 1px solid #444;
        border-radius: 3px;
        cursor: pointer;
    }

    .plot-controls button.selected {
        background-color: #ff965f;
        color: #181b1f;
    }

    .latest-value {
        font-weight: bold;
        min-width: 50px;
        text-align: right;
        margin-right: 20px;
        font-size: 1.2rem;
    }

    .plot {
        flex: 1 1 auto;
        min-height: 0;
        width: 100%;
    }

    .remove-btn {
        position: absolute;
        right: 1px;
        background: transparent;
        border: none;
        color: #ff5555;
        font-size: 1.3rem;
        cursor: pointer;
    }

    .remove-btn:hover {
        color: #ff1111;
    }

    @media (max-width: 1100px) {
        .plot-info h5 {
            font-size: 0.75rem;
        }

        .plot-controls button:last-child {
            display: none;
        }
    }
</style>
