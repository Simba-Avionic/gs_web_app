<script>
    import { onMount, onDestroy, tick } from "svelte";
    import Plotly from "plotly.js-dist-min";

    export let host;
    export let topic;
    export let field;
    export let title;

    let ws;
    let value = null;
    let gaugeDiv;

    function drawGauge() {
        if (!gaugeDiv) return;
        Plotly.newPlot(
            gaugeDiv,
            [
                {
                    type: "indicator",
                    mode: "gauge+number",
                    value: value ?? 0,
                    // title: { text: title },
                    domain: { x: [0, 1], y: [0, 1] },
                    gauge: {
                        axis: { range: field.range ?? [0, 100] },
                        bar: { color: value === null ? "gray" : "green" },
                    },
                },
            ],
            { margin: { t: 0, b: 0 } },
            { displayModeBar: false, displaylogo: false, responsive: true },
        );
    }

    onMount(() => {
        drawGauge();
        ws = new WebSocket(`ws://${host}:8000/${topic}`);

        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data[field] !== undefined) {
                value = data[field];
                drawGauge();
            }
        };
    });

    onDestroy(() => {
        if (ws) ws.close();
    });
</script>

<div bind:this={gaugeDiv} class="gauge-container"></div>

<style>
.gauge-container {
  width: 100%;
  height: 100%;
}
</style>