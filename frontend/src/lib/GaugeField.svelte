<script>
  import { onMount, onDestroy } from "svelte";
  import { subscribeToTopic } from "../js/ws_manager.js";
  import Gauge from "svelte-gauge";

  export let item;

  let unsubscribe;

  let value = null;
  let titleEl;

  function generateTicks(range) {
    const [min, max] = range;
    if (max - min <= 1) {
      return [min, max];
    }
    const step = (max - min) / 2;
    return [min, min + step, max];
  }

  onMount(() => {
    unsubscribe = subscribeToTopic(window.location.host, item.topic, (data) => {
      value = data[item.field];
    });
  });

  onDestroy(() => {
    unsubscribe?.();
  });
</script>

<div class="gauge-container" class:no-data={value === null || value === "None"}>
  <div class="gauge-title">
    <span
      bind:this={titleEl}
    >
      {item.title}
    </span>
  </div>
  <Gauge
    {value}
    start={item.range ? item.range[0] : 0}
    stop={item.range ? item.range[1] : 100}
    stroke={10}
    labels={item.range ? generateTicks(item.range) : []}
    color={value !== null ? "#388729" : "#c41934"}
    segments={item.range ? [item.range] : [[0, 100]]}
    startAngle={60}
    stopAngle={300}
  >
    <div class="gauge-content">
      <span>{Math.round(value)} {item.unit}</span>
    </div>
  </Gauge>
</div>

<style>
  .gauge-title {
    font-size: 0.9em;
    text-align: center;
    padding: 0.55rem;
    white-space: nowrap;
    overflow: hidden;
    position: relative;
  }

  .gauge-container {
    display: flex;
    flex-direction: column;
    box-sizing: border-box;
  }

  .gauge-container :global(svg) {
    max-width: 100%;
    max-height: 100%;
  }

  .gauge-content {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    font-size: calc(var(--gauge-radius) / 5);
    text-align: center;
    font-weight: bold;
  }

  .gauge-container.no-data {
    opacity: 0.6;
  }

  .gauge-container {
    transition: opacity 0.4s ease-in-out;
  }
</style>
