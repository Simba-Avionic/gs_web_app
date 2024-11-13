<script>
  import { onMount, onDestroy } from "svelte";
  import { rosTimeToFormattedTime } from "./lib/Utils.svelte";

  import { slide } from "svelte/transition";
  let showTelemetry = true;

  function toggleTelemetry() {
    showTelemetry = !showTelemetry;
  }

  import { createEventDispatcher } from "svelte";
  const dispatch = createEventDispatcher();

  export let class_name;
  export let topic;
  export let host;

  let temp;
  let telem_data;
  let socket;

  onMount(() => {
    // @ts-ignore
    const socket = new WebSocket(
      `ws://${host}:8000/${String(topic.topic_name)}`,
    );

    socket.onmessage = (event) => {
      temp = JSON.parse(event.data);
      if (temp !== "None" && temp !== null && temp !== undefined) {
        telem_data = temp;
      }
      dispatch("telemetryChange", telem_data);
    };

    socket.onopen = () => {
      console.log(`WebSocket connection for ${topic.topic_name} established`);
    };

    socket.onerror = (error) => {
      console.error("WebSocket error:", error);
    };

    socket.onclose = () => {
      console.log(`WebSocket connection for ${topic.topic_name} is closed`);
    };

    return () => {
      socket.close();
    };
  });

  onDestroy(() => {
    if (socket) {
      socket.close();
    }
  });

  function renderField(telemData, field) {
    const value = telemData[field.val_name];

    // Check if the field is an object (like LoadCell)
    if (typeof value === "object" && value !== null) {
      return Object.keys(value)
        .map((key) => {
          return `
          <div class="${class_name}-nested-field">
            <span>${key}:</span>
            <span>${parseFloat(value[key]).toFixed(2)}</span>
          </div>
        `;
        })
        .join("");
    } else {
      if (field.type.includes("float")) {
        return `<span>${parseFloat(value).toFixed(2)}</span>`;
      } else {
        return `<span>${value}</span>`;
      }
    }
  }
</script>

<div class="field {class_name}-field">
  <div
    class="status-indicator {temp !== 'None' &&
    temp !== null &&
    temp !== undefined
      ? 'green-status'
      : 'red-status'}"
  ></div>
  <div class="field-content {class_name}-field-content">
    <span class="field-text {class_name}-field-text"
      >{String(topic.topic_name)}</span
    >
    {#if telem_data != undefined && telem_data !== "None" && telem_data !== null}
      <span class="timestamp"
        >{rosTimeToFormattedTime(
          telem_data.header.stamp.sec,
          telem_data.header.stamp.nanosec,
        )}</span
      >
      {#if showTelemetry}
        <div
          in:slide={{ duration: 300 }}
          out:slide={{ duration: 300 }}
          class="telemetry-data"
        >
        <div class="fields-column">
          {#each topic.msg_fields as field}
            {#if field.val_name !== "header"}
              <div class="field-value {class_name}-field-value">
                <span>{field.val_name}:</span>
                <span>
                  {@html renderField(telem_data, field)}
                  {#if field.unit}
                    {" " + field.unit}
                  {/if}
                </span>
              </div>
            {/if}
          {/each}
        </div>
          <div class="grafana-panel">
            <!-- <iframe
              title="Grafana Panel"
              src="http://{host}:3001/d-solo/jp2137/simba-dashboard?orgId=1&refresh=1s&from=now-5m&to=now&panelId=tanking/load_cells/combined_raw_kg"
              width="250"
              height="150"
              frameborder="0"
            >
            </iframe> -->
          </div>
        </div>
      {/if}
    {/if}
  </div>
  <button on:click={toggleTelemetry} class="toggle-button">
    {showTelemetry ? "⮝" : "⮟"}
  </button>
</div>

<style>

  .fields-column {
    flex: 1;
  }

  .telemetry-data {
    display: flex;
    gap: 5px;
  }

  .grafana-panel {
    display: flex;
    align-items: center;
  }

  .toggle-button {
    cursor: pointer;
    background: none;
    border: 1px solid rgba(204, 204, 220, 0.5);
    border-radius: 0.5rem;
  }

  .field {
    display: flex;
    align-items: flex-start;
    padding: 12px;
    border-bottom: 1px solid rgba(204, 204, 220, 0.5);
    text-align: left;
    /* justify-content: space-between; */
  }

  .field-content {
    flex: 1;
  }

  .field:last-child {
    border-bottom: none;
  }

  .field:first-child {
    border-top: 1px solid rgba(204, 204, 220, 0.5);
  }

  .field-text {
    color: #ccccdc;
  }

  .field-value span:first-child {
    font-weight: bold;
    margin-right: 4px;
  }

  .gs-field {
    padding: 12px;
  }

  .gs-field-text {
    flex: 0.2;
  }

  .gs-field-value {
    margin-top: 8px;
  }

  .gs-nested-field {
    font-weight: bold;
    padding-left: 50px;
  }

  .rocket-field {
    justify-content: space-between;
    padding: 10px;
  }

  .rocket-field-text {
    min-width: 0;
    overflow: hidden;
    white-space: nowrap;
    text-overflow: ellipsis;
  }

  .rocket-field-value {
    margin-top: 5px;
  }

  .status-indicator {
    width: 16px;
    height: 16px;
    border-radius: 50%;
    margin-right: 8px;
  }

  .timestamp {
    margin-left: 8px;
    color: rgba(204, 204, 220, 0.65);
    font-size: 0.8em;
  }

  .green-status {
    background: linear-gradient(90deg, #7fff7f, #5eff5e, #3dff3d, #1aff1a);
    background-size: 100% 100%;
    animation: gradientAnimation 3s ease infinite;
  }

  .orange-status {
    background: linear-gradient(45deg, orange, yellow);
    background-size: 100% 100%;
    animation: orangeGradientAnimation 3s ease infinite;
  }

  .red-status {
    background: linear-gradient(45deg, red, pink);
    background-size: 100% 100%;
    animation: redGradientAnimation 3s ease infinite;
  }

  @keyframes gradientAnimation {
    0% {
      background-position: 0% 50%;
    }
    50% {
      background-position: 100% 50%;
    }
    100% {
      background-position: 0% 50%;
    }
  }
</style>
