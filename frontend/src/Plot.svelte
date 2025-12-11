<script>
    // @ts-nocheck

    import { onMount, onDestroy } from "svelte";
    import Plotly from "plotly.js-dist-min";
    import { fetchConfig, rosTimeToFormattedTime } from "./lib/Utils.svelte";
    import { theme } from "./js/theme.js";
    import { cssVar, lightThemeColors, darkThemeColors } from "./js/colors.js";
    import { getPlotLayout } from "./js/plot_config.js";

    export let host;
    export let field;
    export let time_range = 1;
    export let onRemove;

    let color;
    let resizeHandler;
    let plotDiv;
    let latestValue = "--";
    let ws;
    let currentTheme;
    let buffer = [];
    let flushInterval;

    $: $theme, updateTheme();

    let xData = [];
    let yData = [];

    function updateTheme() {
        if (!plotDiv || !plotDiv.hasChildNodes()) return;
        // Build a minimal relayout object containing only theme-related keys.
        // This avoids clobbering runtime axis ranges / zoom / autoscale state.
        const layout = getPlotLayout(field);

        const relayoutObj = {};

        if (layout.plot_bgcolor !== undefined)
            relayoutObj["plot_bgcolor"] = layout.plot_bgcolor;
        if (layout.paper_bgcolor !== undefined)
            relayoutObj["paper_bgcolor"] = layout.paper_bgcolor;
        if (layout.font && layout.font.color)
            relayoutObj["font.color"] = layout.font.color;

        // xaxis visual tweaks
        if (layout.xaxis) {
            if (layout.xaxis.tickfont)
                relayoutObj["xaxis.tickfont"] = layout.xaxis.tickfont;
            if (layout.xaxis.gridcolor)
                relayoutObj["xaxis.gridcolor"] = layout.xaxis.gridcolor;
            if (layout.xaxis.tickformat)
                relayoutObj["xaxis.tickformat"] = layout.xaxis.tickformat;
        }

        // yaxis visual tweaks (including tick text/vals for bools)
        if (layout.yaxis) {
            if (layout.yaxis.tickfont)
                relayoutObj["yaxis.tickfont"] = layout.yaxis.tickfont;
            if (layout.yaxis.gridcolor)
                relayoutObj["yaxis.gridcolor"] = layout.yaxis.gridcolor;
            if (layout.yaxis.tickvals)
                relayoutObj["yaxis.tickvals"] = layout.yaxis.tickvals;
            if (layout.yaxis.ticktext)
                relayoutObj["yaxis.ticktext"] = layout.yaxis.ticktext;
        }

        // If we didn't detect anything to update, skip the relayout call.
        if (Object.keys(relayoutObj).length === 0) return;

        Plotly.relayout(plotDiv, relayoutObj);
    }

    function getRandomColor() {
        if (currentTheme === "dark") {
            return darkThemeColors[
                Math.floor(Math.random() * darkThemeColors.length)
            ];
        } else {
            return lightThemeColors[
                Math.floor(Math.random() * lightThemeColors.length)
            ];
        }
    }

    function setTimeRange(min) {
        time_range = min;
        closeWebSocket();
        fetchData();
    }

    async function fetchData() {
        try {
            const res = await fetch(
                `http://${host}/${field.topic}/query?field_name=${field.parent ? `${field.parent}/${field.val_name}` : field.val_name}&time_range=${time_range}`,
            );
            const data = await res.json();

            // Raw x/y from server (ISO timestamps expected)
            let rawX = data.records.map((entry) =>
                new Date(entry._time).toISOString(),
            );
            let rawY = data.records.map((entry) => entry._value);

            if (field.type === "bool") {
                rawY = rawY.map((v) => (v ? 1 : 0));
            }

            latestValue = rawY.length > 0 ? rawY[rawY.length - 1] : "--";

            // Compute time-window (ms) relative to newest sample we received
            const newestMs = rawX.length
                ? Date.parse(rawX[rawX.length - 1])
                : Date.now();
            const windowStartMs = newestMs - time_range * 60 * 1000;

            // Filter by timestamp (time-based window), preserve order
            const paired = rawX.map((x, i) => ({
                tMs: Date.parse(x),
                tIso: x,
                v: rawY[i],
            }));
            const filtered = paired.filter((p) => p.tMs >= windowStartMs);

            // If nothing falls into the window (rare), fall back to the most recent N samples
            const finalPairs = filtered.length
                ? filtered
                : paired.slice(
                      -Math.min(
                          paired.length,
                          Math.max(1, Math.floor(time_range * 60)),
                      ),
                  );

            xData = finalPairs.map((p) => p.tIso);
            yData = finalPairs.map((p) => p.v);

            Plotly.newPlot(
                plotDiv,
                [
                    {
                        x: xData,
                        y: yData,
                        type: "scatter",
                        mode: "lines",
                        line: { color: color, width: 1.5 },
                        name: field,
                        hoverinfo: "x+y",
                    },
                ],
                getPlotLayout(field),
                {
                    responsive: true,
                    displayModeBar: false,
                },
            );

            // Enforce x-axis range to the time window we've selected
            if (xData.length) {
                const startIso = new Date(Date.parse(xData[0])).toISOString();
                const endIso = new Date(
                    Date.parse(xData[xData.length - 1]),
                ).toISOString();
                Plotly.relayout(plotDiv, { "xaxis.range": [startIso, endIso] });
            }

            openWebSocket();
        } catch (error) {
            console.error("Error fetching or plotting data:", error);
        }
    }

    function openWebSocket() {
        ws = new WebSocket(`ws://${host}/${field.topic}`);

        ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                let v;

                try {
                    v = field.parent
                        ? msg[field.parent][field.val_name]
                        : msg[field.val_name];
                } catch (err) {
                    return;
                }

                if (v === undefined) return;

                if (field.type === "bool") {
                    v = v ? 1 : 0;
                }

                const t = rosTimeToFormattedTime(
                    true,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                );

                buffer.push({ t, v });
            } catch (err) {
                console.error("WS parse error:", err);
            }
        };

        ws.onclose = () => closeWebSocket();

        flushInterval = setInterval(() => {
            if (buffer.length === 0) return;

            const last = buffer[buffer.length - 1];
            buffer = [];

            // append latest
            xData.push(last.t);
            yData.push(last.v);
            latestValue = last.v;

            // Compute time window based on last timestamp
            const lastMs = Date.parse(last.t);
            const windowStartMs = lastMs - time_range * 60 * 1000;

            // Filter existing arrays by time window using paired array
            const paired = xData.map((x, i) => ({
                tMs: Date.parse(x),
                tIso: x,
                v: yData[i],
            }));
            const filtered = paired.filter((p) => p.tMs >= windowStartMs);

            // If nothing matches (edge cases), keep last few samples
            const finalPairs = filtered.length
                ? filtered
                : paired.slice(
                      -Math.min(
                          paired.length,
                          Math.max(1, Math.floor(time_range * 60)),
                      ),
                  );

            // Replace arrays with filtered results
            xData = finalPairs.map((p) => p.tIso);
            yData = finalPairs.map((p) => p.v);

            // Extend trace with the new point (no maxPoints param — we trim arrays explicitly)
            try {
                Plotly.extendTraces(plotDiv, { x: [[last.t]], y: [[last.v]] }, [
                    0,
                ]);
                // Explicitly set x-axis window to time_range minutes ending at last.t
                Plotly.relayout(plotDiv, {
                    "xaxis.range": [
                        new Date(windowStartMs).toISOString(),
                        last.t,
                    ],
                });
            } catch (err) {
                console.warn("Plotly update error:", err);
            }
        }, 1000);
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
                class:selected={time_range === 2}
                on:click={() => setTimeRange(2)}>2m</button
            >
            <button
                class:selected={time_range === 5}
                on:click={() => setTimeRange(5)}>5m</button
            >
            <button
                class:selected={time_range === 10}
                on:click={() => setTimeRange(10)}>10m</button
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
                ? latestValue.toFixed(2)
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
        .plot-info {
            display: none;
        }
    }
</style>
