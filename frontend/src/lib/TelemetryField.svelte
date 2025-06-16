<script>
  import { onMount, onDestroy, createEventDispatcher } from "svelte";
  import { rosTimeToFormattedTime, stripSimbaPrefix } from "./Utils.svelte";

  import { slide } from "svelte/transition";
  let showTelemetry = false;

  function toggleTelemetry() {
    showTelemetry = !showTelemetry;
  }

  const dispatch = createEventDispatcher();

  export let topic;
  export let host;

  let temp;
  let telem_data;
  let socket;

  onMount(() => {
    socket = new WebSocket(`ws://${host}:8000/${String(topic.topic_name)}`);

    socket.onmessage = (event) => {
      temp = JSON.parse(event.data);
      if (temp !== "None" && temp !== null && temp !== undefined) {
        telem_data = temp;
      }
      dispatch("telemetryChange", telem_data);
    };

    socket.onopen = () => {
      console.log(`Connected to ${topic.topic_name}`);
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
          <div class="nested-field">
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

<div class="field">
  <div
    class="status-indicator {temp !== 'None' &&
    temp !== null &&
    temp !== undefined
      ? 'green-status'
      : 'red-status'}"
  ></div>
  <div class="field-content">
    <div class="field-header">
      <span class="field-text">{String(stripSimbaPrefix(topic.topic_name))}</span>
      {#if telem_data != undefined && telem_data !== "None" && telem_data !== null}
        <span class="timestamp"
          >{rosTimeToFormattedTime(
            telem_data.header.stamp.sec,
            telem_data.header.stamp.nanosec,
          )}</span
        >
      {/if}
    </div>

    {#if telem_data != undefined && telem_data !== "None" && telem_data !== null}
      {#if showTelemetry}
        <div
          in:slide={{ duration: 333 }}
          out:slide={{ duration: 333 }}
          class="telemetry-data"
        >
          <div class="fields-column">
            {#each topic.msg_fields as field}
              {#if field.val_name !== "header"}
                <div class="field-value">
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
    {showTelemetry ? "▲" : "▼"}
  </button>
</div>

<style>
  .field-header {
    display: flex;
    flex-direction: column;
  }

  .fields-column {
    flex: 1;
  }

  .telemetry-data {
    display: flex;
    gap: 5px;
  }

  .field {
    display: flex;
    align-items: flex-start;
    padding: 12px;
    border-bottom: 1px solid rgba(204, 204, 220, 0.5);
    text-align: left;
    min-width: 0;
    width: 100%;
    box-sizing: border-box;
    border-bottom: none;
    margin: 0;
  }

  .field-content {
    margin: 0 5px;
    flex: 1;
  }

  .field {
    border-top: 1px solid rgba(204, 204, 220, 0.5);
  }

  .field-value span:first-child {
    font-weight: bold;
    margin-right: 4px;
  }

  .field-value {
    margin-top: 5px;
    white-space: normal;
    overflow-wrap: break-word;
    word-break: break-word;
    width: 100%;
  }

  .field-value span {
    max-width: 100%;
    display: inline-block;
  }

  .nested-field {
    font-weight: bold;
    padding-left: 50px;
  }

  .timestamp {
    color: rgba(204, 204, 220, 0.65);
    font-size: 0.8em;
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
</style>
