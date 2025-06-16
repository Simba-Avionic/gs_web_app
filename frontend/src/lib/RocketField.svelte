<script>
  import { onMount, onDestroy, createEventDispatcher } from "svelte";
  import { rosTimeToFormattedTime, getStatusString } from "./Utils.svelte";

  import { slide } from "svelte/transition";
  let showTelemetry = false;
  let temp;
  let telem_data;
  let topicStatusMap = {};
  let allTopicsOffline = true;
  let someTopicsOffline = false;

  function toggleTelemetry() {
    showTelemetry = !showTelemetry;
  }
  const dispatch = createEventDispatcher();

  const topicDataMap = {
    Recovery: [
      {
        topic: "mavlink/simba_actuator",
        fields: [
          {
            name: "line_cutter",
            extract: (data) => (data.values & 0x08 ? "OPEN" : "CLOSED"),
            display: "Line Cutter",
          },
          {
            name: "rec_valve",
            extract: (data) => (data.values & 0x10 ? "OPEN" : "CLOSED"),
            display: "REC Valve",
          },
        ],
      },
    ],
    Avionics: [
      {
        topic: "mavlink/simba_heartbeat",
        fields: [
          {
            name: "flight_computer_status",
            extract: (data) => getStatusString(data.flight_computer_status),
            display: "Status",
          },
        ],
      },
    ],
    "Tank Pressure": [
      {
        topic: "mavlink/simba_tank_pressure",
        fields: [
          {
            name: "pressure",
            extract: (data) => data.pressure,
            display: "Pressure",
            unit: "bar",
          },
          {
            name: "delta_pressure",
            extract: (data) => data.d_pressure,
            display: "Delta pressure",
            unit: "bar",
          },
        ],
      },
    ],
    "Tank Temperature": [
      {
        topic: "mavlink/simba_tank_temperature",
        fields: [
          {
            name: "temperature1",
            extract: (data) => data.temp1,
            display: "Temperature1",
            unit: "°C",
          },
          {
            name: "temperature2",
            extract: (data) => data.temp2,
            display: "Temperature2",
            unit: "°C",
          },
          {
            name: "temperature3",
            extract: (data) => data.temp3,
            display: "Temperature3",
            unit: "°C",
          },
        ],
      },
    ],
    "Tank Actuators": [
      {
        topic: "mavlink/simba_actuator",
        fields: [
          {
            name: "main_valve",
            extract: (data) => (data.values & 0x02 ? "OPEN" : "CLOSED"),
            display: "Main Valve",
          },
          {
            name: "tank_vent",
            extract: (data) => (data.values & 0x04 ? "OPEN" : "CLOSED"),
            display: "Tank Vent",
          },
        ],
      },
    ],
    Engine: [
      {
        topic: "mavlink/simba_heartbeat",
        fields: [
          {
            name: "engine_computer_status",
            extract: (data) => getStatusString(data.engine_computer_status),
            display: "Engine Computer Status",
          },
        ],
      },
      {
        topic: "mavlink/simba_actuator",
        fields: [
          {
            name: "primer",
            extract: (data) => (data.values & 0x01 ? "ON" : "OFF"),
            display: "Igniter",
          },
        ],
      },
    ],
  };

  export let host;
  export let title;

  let relevantTopicNames = [];
  let extractedData = {};
  let lastUpdated = null;
  let sockets = {};
  let isConnected = false;
  let hasNonOkStatus = false;

  function findRelevantTopicNames() {
    if (!topicDataMap[title]) return [];

    return topicDataMap[title]
      .map((topicConfig) => topicConfig.topic)
      .filter((value, index, self) => self.indexOf(value) === index); // Unique values
  }

  function processTopicData(topicName, telem_data) {
    if (!topicDataMap[title]) return;

    topicStatusMap[topicName] = true;
    updateTopicStatusFlags();

    let foundNonOkStatus = false;

    const topicConfigs = topicDataMap[title].filter(
      (config) => config.topic === topicName
    );

    for (const config of topicConfigs) {
      for (const field of config.fields) {
        try {
          const extractedValue = field.extract(telem_data);
          if (
            field.name.includes("status") &&
            extractedValue !== "OK" &&
            typeof extractedValue === "string"
          ) {
            foundNonOkStatus = true;
          }

          if (!extractedData[field.name]) {
            extractedData[field.name] = {};
          }
          extractedData[field.name] = {
            value: extractedValue,
            display: field.display || field.name,
            unit: field.unit || "",
          };
        } catch (e) {
          console.error(`Error extracting ${field.name} from ${topicName}:`, e);
        }
      }
    }

    hasNonOkStatus = foundNonOkStatus;
    try {
      lastUpdated = {
        sec: telem_data.header.stamp.sec,
        nanosec: telem_data.header.stamp.nanosec,
      };
    } catch (e) {
      lastUpdated = null;
    }
  }

  function updateTopicStatusFlags() {
    // Check if all topics are offline
    allTopicsOffline = relevantTopicNames.length > 0 && 
      relevantTopicNames.every(topic => !topicStatusMap[topic]);
    
    // Check if some (but not all) topics are offline
    someTopicsOffline = relevantTopicNames.some(topic => !topicStatusMap[topic]) && 
      !allTopicsOffline;
  }

  function setupWebSockets() {
    relevantTopicNames.forEach((topicName) => {
      topicStatusMap[topicName] = false;
      
      const socket = new WebSocket(`ws://${host}:8000/${topicName}`);

      socket.onopen = () => {
        console.log(`Connected to ${topicName}`);
        isConnected = true;
      };

      socket.onmessage = (event) => {
        try {
          temp = JSON.parse(event.data);
          if (temp !== "None" && temp !== null && temp !== undefined) {
            telem_data = temp;
            processTopicData(topicName, telem_data);
          } else{
            topicStatusMap[topicName] = false;
            updateTopicStatusFlags();
          }

          dispatch("telemetryChange", { telem_data, hasNonOkStatus });
        } catch (e) {
          console.error(`Error processing telem_data from ${topicName}:`, e);
        }
      };

      socket.onclose = () => {
        console.log(`Disconnected from ${topicName}`);
        topicStatusMap[topicName] = false;
        
        // Update status flags
        updateTopicStatusFlags();
        if (Object.values(sockets).every((s) => s.readyState > 1)) {
          isConnected = false;
        }
      };

      sockets[topicName] = socket;
    });

    updateTopicStatusFlags();
  }

  onMount(() => {
    relevantTopicNames = findRelevantTopicNames();
    setupWebSockets();
  });

  onDestroy(() => {
    Object.values(sockets).forEach((socket) => {
      if (socket && socket.readyState < 2) {
        socket.close();
      }
    });
  });
</script>

<div class="field">
  <div class="field-top-row">
    <div
      class="status-indicator {allTopicsOffline
      ? 'red-status'
      : someTopicsOffline || hasNonOkStatus
        ? 'orange-status'
        : 'green-status'}"
    ></div>

    <div class="field-header">
      <span class="field-text">{String(title)}</span>
      {#if telem_data != undefined && telem_data !== "None" && telem_data !== null}
        <span class="timestamp">
          {rosTimeToFormattedTime(lastUpdated.sec, lastUpdated.nanosec)}
        </span>
      {/if}
    </div>

    <button on:click={toggleTelemetry} class="toggle-button">
      {showTelemetry ? "▲" : "▼"}
    </button>
  </div>

  {#if telem_data != undefined && telem_data !== "None" && telem_data !== null && showTelemetry}
    <div
      in:slide={{ duration: 333 }}
      out:slide={{ duration: 333 }}
      class="telemetry-data"
    >
      <div class="fields-column">
        {#each Object.entries(extractedData) as [fieldName, fieldInfo]}
          <div class="field-value">
            <span class="field-label">{fieldName}:</span>
            <span class="field-data">
              {typeof fieldInfo.value === "number"
                ? Number.isInteger(fieldInfo.value)
                  ? fieldInfo.value
                  : fieldInfo.value.toFixed(2)
                : fieldInfo.value}
              {fieldInfo.unit}
            </span>
          </div>
        {/each}
      </div>
    </div>
  {/if}
</div>

<style>
  .field {
    display: flex;
    flex-direction: column;
    width: 100%;
    text-align: left;
    box-sizing: border-box;
  }

  .field-top-row {
    display: flex;
    width: 100%;
    align-items: center;
  }

  .field-header {
    display: flex;
    flex-direction: row;
    align-items: center;
    flex-grow: 1;
    justify-content: space-between;
    margin-right: 0.75rem;
  }

  .field-text {
    font-weight: bold;
  }

  .timestamp {
    color: rgba(204, 204, 220, 0.65);
    font-size: 0.8em;
    margin-left: 8px;
  }

  .toggle-button {
    cursor: pointer;
    background: none;
    border: 1px solid rgba(204, 204, 220, 0.5);
    border-radius: 0.5rem;
    align-self: flex-start;
  }

  .telemetry-data {
    font-size: 0.95rem;
    padding-left: 0.5rem;
    width: 100%;
    box-sizing: border-box;
  }

  .fields-column {
    width: 100%;
  }

  .field-value {
    margin-top: 0.5rem;
    display: flex;
    justify-content: space-between;
    width: 100%;
  }

  .field-label {
    font-weight: bold;
    margin-right: 4px;
  }

  .field-data {
    text-align: right;
  }

  @media (min-width: 1920px) {
    .field {
      padding: 0.5rem;
      font-size: 1rem;
    }
    .telemetry-data {
      margin-top: 0.5rem;
      font-size: 1rem;
    }
    .field-top-row {
      font-size: 1.2rem;
    }
    .status-indicator {
      margin-right: 1.5rem;
    }
  }

  @media (max-width: 1280px) {
    .status-indicator {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      margin-right: 8px;
    }
  }
</style>
