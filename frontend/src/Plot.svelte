<script>
    // @ts-nocheck

    import { onMount, onDestroy } from "svelte";
    import Plotly from "plotly.js-dist-min";
    import { fetchConfig, rosTimeToFormattedTime } from "./lib/Utils.svelte";
    import { theme } from "./js/theme.js";
    import { cssVar, lightThemeColors, darkThemeColors } from "./js/colors.js";
    
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

    $: $theme, updateTheme();

    let xData = [];
    let yData = [];

    function updateTheme() {
        if (!plotDiv || !plotDiv.hasChildNodes()) return;

        const layoutUpdate = {
            plot_bgcolor: cssVar("--snd-bg-color"),
            font: { color: cssVar("--text-color") },
            xaxis: {
                gridcolor: cssVar("--nav-active"),
                zeroline: false,
            },
            yaxis: {
                gridcolor: cssVar("--nav-active"),
                zeroline: false,
            },
        };
        Plotly.relayout(plotDiv, layoutUpdate);
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

    function generateTicks(range) {
        const [min, max] = range;
        const step = (max - min) / 2; // divide into 3 intervals -> 4 ticks
        return [min, min + step, max];
    }

    function setTimeRange(min) {
        time_range = min;
        closeWebSocket();
        fetchData(); // restart with new range
    }

    async function fetchData() {
        try {
            const res = await fetch(
                `http://${host}:8000/${field.topic}/query?field_name=${field.val_name}&time_range=${time_range}`,
            );
            const data = await res.json();

            xData = data.records.map((entry) =>
                new Date(entry._time).toISOString(),
            );
            yData = data.records.map((entry) => entry._value);

            latestValue = yData.length > 0 ? yData[yData.length - 1] : "--";

            const maxPoints = 700;
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
                {
                    width: "100%",
                    margin: { t: 10, b: 40, l: 0, r: 0 },
                    title: "",
                    xaxis: {
                        title: "Time",
                        type: "date",
                        tickformat: "%H:%M:%S",
                        tickfont: { size: 10 },
                        showgrid: true,
                        zeroline: false,
                        gridcolor: cssVar("--nav-active"),
                    },
                    yaxis: {
                        title: {
                            text: `${field.val_name} ${field.unit ? `(${field.unit})` : ""}`,
                            standoff: 20,
                            font: { size: 13 },
                        },
                        // TODO: automatic range detection
                        range: field.range || [0, 100],
                        tickvals: generateTicks(field.range || [0, 100]),

                        tickfont: { size: 10 },
                        showgrid: true,
                        zeroline: false,
                        automargin: true,
                        gridcolor: cssVar("--nav-active"),
                    },
                    plot_bgcolor: cssVar("--snd-bg-color"),
                    paper_bgcolor: "rgba(0,0,0,0)",
                    font: {
                        color: cssVar("--text-color"),
                        family: "Inter, system-ui, Helvetica, sans-serif",
                    },
                    autosize: true,
                },
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
        ws = new WebSocket(`ws://${host}:8000/${field.topic}`);
        ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);

                const t = rosTimeToFormattedTime(
                    true,
                    msg.header.stamp.sec,
                    msg.header.stamp.nanosec,
                );

                const v = msg[field.val_name];

                xData.push(t);
                yData.push(v);
                latestValue = v;

                const maxPoints = time_range * 60;
                if (xData.length > maxPoints) {
                    xData.shift();
                    yData.shift();
                }

                Plotly.extendTraces(
                    plotDiv,
                    { x: [[t]], y: [[v]] },
                    [0],
                    maxPoints,
                );
            } catch (err) {
                console.error("WS parse error:", err);
            }
        };

        ws.onclose = () => {
            console.log("WebSocket closed");
        };
    }

    function closeWebSocket() {
        if (ws) {
            ws.close();
            ws = null;
        }
    }

    onMount(() => {
        color = getRandomColor();
        fetchData();
        resizeHandler = () => {
            Plotly.Plots.resize(plotDiv);
        };
        window.addEventListener("resize", resizeHandler);
    });

    onDestroy(() => {
        closeWebSocket();
        window.removeEventListener("resize", resizeHandler);
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
            <h5>{field.msg_type} — {field.val_name}</h5>
        </div>
        <div class="latest-value">
            {typeof latestValue === "number"
                ? latestValue.toFixed(2)
                : latestValue}
            {field.unit ? field.unit : ""}
        </div>
        <button class="remove-btn" on:click={onRemove} aria-label="Remove plot"
            >×</button
        >
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
