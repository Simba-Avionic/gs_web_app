<script>
// @ts-nocheck

    import { onMount, onDestroy } from "svelte";
    import Plotly from "plotly.js-dist-min";
    import {colors, cssVar} from "./colors.js";

    export let host;
    export let msg_type;
    export let topic;
    export let field;
    export let time_range = 1;

    let plotDiv;
    let latestValue = "--";
    let intervalId;
    let color;

    let xData = [];
    let yData = [];

    function getRandomColor() {
        return colors[Math.floor(Math.random() * colors.length)];
    }


    function setTimeRange(min) {
        time_range = min;
        restartInterval();
    }

    async function fetchData() {
        try {
            const res = await fetch(
                `http://${host}:8000/${topic}/query?field_name=${field}&time_range=${time_range}`,
            );
            const data = await res.json();

            xData = data.records.map((entry) =>
                new Date(entry._time).toISOString(),
            );
            yData = data.records.map((entry) => entry._value);

            latestValue = yData.length > 0 ? yData[yData.length - 1] : "--";

            const maxPoints = 300;
            if (xData.length > maxPoints) {
                xData = xData.slice(-maxPoints);
                yData = yData.slice(-maxPoints);
            }

            Plotly.react(
                plotDiv,
                [
                    {
                        x: xData,
                        y: yData,
                        type: "scatter",
                        mode: "lines",
                        line: { color: color, width: 1.5 },
                        name: "Abort Flag",
                        hoverinfo: "x+y",
                    },
                ],
                {
                    margin: { t: 20, b: 30, l: 30, r: 20 },
                    xaxis: {
                        title: "Time",
                        type: "date",
                        tickformat: "%H:%M:%S",
                        tickfont: { size: 10 },
                        showgrid: true,
                        zeroline: false,
                    },
                    yaxis: {
                        range: [0, 100],
                        tickvals: [0, 25, 50, 75, 100],
                        tickfont: { size: 10 },
                        showgrid: true,
                        zeroline: false,
                        automargin: true,
                    },
                    plot_bgcolor: cssVar('--snd-bg-color'),
                    paper_bgcolor: "rgba(0,0,0,0)",
                    font: { color: cssVar('--text-color') },
                    autosize: true
                },
                { responsive: true }
            );
        } catch (error) {
            console.error("Error fetching or plotting data:", error);
        }
    }

    function restartInterval() {
        clearInterval(intervalId);
        fetchData();
        intervalId = setInterval(fetchData, 1000);
    }

    onMount(() => {
        restartInterval();

        color = getRandomColor();

        window.addEventListener("resize", () => {
            Plotly.Plots.resize(plotDiv);
        });
    });

    onDestroy(() => {
        clearInterval(intervalId);
        window.removeEventListener("resize", Plotly.Plots.resize);
    });
</script>

<div class="flex-container">
    <div class="plot-header">
        <div class="plot-info">
            <h5>{msg_type} — {field}</h5>
        </div>
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
        </div>
        <div class="latest-value">
            {typeof latestValue === "number"
                ? latestValue.toFixed(2)
                : latestValue}
        </div>
    </div>

    <div class="plot" bind:this={plotDiv}></div>
</div>

<style>
    .flex-container {
        display: flex;
        flex-direction: column;
        height: 100%;
    }

    .plot-header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        /* padding: 4px 8px; */
        background-color: var(--snd-bg-color);
        color: var(--text-color);
        font-size: 0.85rem;
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
        color: black;
    }

    .latest-value {
        font-weight: bold;
        min-width: 50px;
        text-align: right;
    }

    .plot {
        flex-grow: 1;
        width: 100%;
        height: 100%;
    }
</style>
