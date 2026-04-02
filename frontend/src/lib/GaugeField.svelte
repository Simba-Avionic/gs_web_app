<script>
  import { onMount, onDestroy } from "svelte";
  import { subscribeToTopic } from "../js/ws_manager.js";
  import { getStateString } from "./Utils.svelte";
  import Gauge from "svelte-gauge";

  export let item;

  let unsubscribe;

  let value = null;
  let titleEl;

  $: isOutOfRange =
    item.range &&
    value !== null &&
    value !== undefined &&
    (value < item.range[0] || value > item.range[1]);

  $: gaugeColor =
    value === null || value === undefined
      ? "#c41934" // Red: No data
      : isOutOfRange
        ? "#fa6400" // Orange: Status is out of range
        : "#388729"; // Green: Normal / In range

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

<div
  class="gauge-container"
  class:no-data={value === null || value === undefined}
>
  <div class="gauge-title">
    <span bind:this={titleEl}>
      {item.title}
    </span>
  </div>
  <Gauge
    {value}
    start={item.range ? item.range[0] : 0}
    stop={item.range ? item.range[1] : 100}
    stroke={10}
    labels={item.range ? generateTicks(item.range) : []}
    color={gaugeColor}
    segments={item.range ? [item.range] : [[0, 100]]}
    startAngle={item.type === "status" ? 90 : 60}
    stopAngle={item.type === "status" ? 270 : 300}
  >
    <div class="gauge-content" class:is-status={item.type === "status"}>
      {#if item.type === "status"}
      
        <span class="status-text">
          {value !== null && value !== undefined
            ? getStateString(value, item.enum).replace(/_/g, "\n")
            : "---"}
        </span>
      {:else}
        <span>
          {typeof value === "number"
            ? Number.isInteger(value)
              ? value
              : value.toFixed(1)
            : (value ?? "---")}
          {item.unit ?? ""}
        </span>
      {/if}
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
    font-weight: bold;
  }

  .gauge-container {
    display: flex;
    flex-direction: column;
    box-sizing: border-box;
    transition: opacity 0.4s ease-in-out;
  }

  .gauge-container.no-data {
    opacity: 0.6;
  }

  .gauge-container :global(svg) {
    max-width: 100%;
    max-height: 100%;
  }

  .gauge-container :global(svg path) {
    transition: stroke 0.4s ease-in-out;
  }

  .gauge-container :global(.gauge-slot-container) {
    margin: 0 !important; /* Replace 0 with whatever margin you actually want */
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

  .status-text {
    white-space: pre-line;
    display: block;
    text-align: center;
    line-height: 1.2;
    font-size: 0.95em;
  }
</style>
