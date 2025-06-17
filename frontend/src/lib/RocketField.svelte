<script>
  import BaseField from "./BaseField.svelte";
  import { getStatusString } from "./Utils.svelte";

  export let host;
  export let title;

  let isExpanded = false;
  let extractedData = {};
  let hasNonOkStatus = false;

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

  const relevantTopics = topicDataMap[title]
    ? topicDataMap[title]
        .map((config) => config.topic)
        .filter((value, index, self) => self.indexOf(value) === index)
    : [];

  function processData(topicName, data) {
    let foundNonOkStatus = false;

    const topicConfigs =
      topicDataMap[title]?.filter((config) => config.topic === topicName) || [];

    for (const config of topicConfigs) {
      for (const field of config.fields) {
        try {
          const extractedValue = field.extract(data);
          if (
            field.name.includes("status") &&
            extractedValue !== "OK" &&
            typeof extractedValue === "string"
          ) {
            foundNonOkStatus = true;
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

    if (hasNonOkStatus !== foundNonOkStatus) {
      hasNonOkStatus = foundNonOkStatus;
    }
  }
</script>

<BaseField
  {title}
  {host}
  topicNames={relevantTopics}
  bind:isExpanded
  {processData}
  className="rocket-telem-class"
  dataWarning={hasNonOkStatus}
  on:statusChange={({ detail }) => {
    // Handle status changes if needed
  }}
>
  <!-- Custom content inside the expanded panel -->
  <div class="fields-column">
    {#each Object.entries(extractedData) as [fieldName, fieldInfo]}
      <div class="field-value">
        <span class="field-label">{fieldInfo.display}:</span>
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
</BaseField>

<style>
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

  :global(.rocket-telem-class .timestamp) {
    font-size: 0.8rem;
  }

  :global(.rocket-telem-class .status-indicator) {
    width: 12px;
    height: 12px;
    border-radius: 50%;
  }

  @media (min-width: 1920px) {
    :global(.rocket-telem-class) {
      padding: 0.5rem;
    }
    :global(.rocket-telem-class .telemetry-data) {
      font-size: 1rem;
    }
    :global(.rocket-telem-class .field-top-row) {
      font-size: 1.25rem;
    }
    :global(.rocket-telem-class .status-indicator) {
      margin-right: 1rem;
    }
  }

  @media (max-width: 1280px) {
    :global(.rocket-telem-class .status-indicator) {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      margin-right: 8px;
    }
  }
</style>
