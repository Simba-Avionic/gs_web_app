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
        Plotly.relayout(plotDiv, getPlotLayout(field));
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

            xData = data.records.map((entry) =>
                new Date(entry._time).toISOString(),
            );
            yData = data.records.map((entry) => entry._value);

            if (field.type === "bool") {
                yData = yData.map((v) => (v ? 1 : 0));
            }

            latestValue = yData.length > 0 ? yData[yData.length - 1] : "--";

            const maxPoints = time_range * 60;
            if (xData.length > maxPoints) {
                xData = xData.slice(-maxPoints);
                yData = yData.slice(-maxPoints);
            }

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

                // TESTING ...
                const stamp = msg.header.stamp;
                const msgTimeMs = stamp.sec * 1000 + stamp.nanosec / 1e6;
                const nowMs = Date.now(); // aktualny czas w ms
                const delayMs = nowMs - msgTimeMs; // opóźnienie w ms
                console.log(`Opóźnienie: ${delayMs.toFixed(2)} ms`);

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

            xData.push(last.t);
            yData.push(last.v);
            latestValue = last.v;

            const maxPoints = time_range * 60;
            if (xData.length > maxPoints) {
                xData.shift();
                yData.shift();
            }

            Plotly.extendTraces(
                plotDiv,
                { x: [[last.t]], y: [[last.v]] },
                [0],
                maxPoints,
            );
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
        color = getRandomColor();
        fetchData();

        // const fetchInterval = setInterval(() => {
        //     fetchData();
        // }, 1000);
    });

    onDestroy(() => {
        closeWebSocket();
        clearInterval(fetchInterval);
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
