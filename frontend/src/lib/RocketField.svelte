<script>
  import BaseField from "./BaseField.svelte";
  import { getStatusString } from "./Utils.svelte";

  export let title;

  let isExpanded = true;
  let extractedData = {};
  let hasNonOkStatus = false;

  const topicDataMap = {
    Recovery: [
      {
        topic: "mavlink/simba_rocket_heartbeat",
        fields: [
          {
            name: "recovery_servo",
            extract: (data) => (data.values & 0x08 ? "OPEN" : "CLOSED"),
            display: "Recovery Servo",
          },
          {
            name: "line_cutter_act",
            extract: (data) => (data.values & 0x10 ? "ON" : "OFF"), // 0x10 is 16
            display: "Linecutter",
          },
        ]
      }
    ],
    Avionics: [
      {
        topic: "mavlink/simba_rocket_heartbeat",
        fields: [
          {
            name: "flight_computer_state",
            extract: (data) => getStatusString(data.flight_computer_state),
            display: "Flight Computer State",
          },
          {
            name: "cameras_act",
            extract: (data) => (data.values & 0x20 ? "ON" : "OFF"), // 0x20 is 32
            display: "Cameras",
          }
        ],
      },
      {
        topic: "mavlink/simba_computer_temperature",
        fields: [
          {
            name: "mb_temp",
            extract: (data) => data.mb_temp,
            display: "Computer temp",
            unit: "°C"
          }
        ],
      },
      {
      }
    ],
    "Tank Sensors": [
      {
        topic: "mavlink/simba_tank_temperature",
        fields: [
          {
            name: "upper_tank",
            extract: (data) => data.upper_tank,
            display: "Upper Tank Temp",
            unit: "°C",
          },
          {
            name: "middle_tank",
            extract: (data) => data.middle_tank,
            display: "Middle Tank Temp",
            unit: "°C",
          },
          {
            name: "lower_tank",
            extract: (data) => data.lower_tank,
            display: "Lower Tank Temp",
            unit: "°C",
          },
        ],
      },
      {
        topic: "mavlink/simba_tank_pressure",
        fields: [
          {
            name: "pressure",
            extract: (data) => data.pressure,
            display: "Pressure",
            unit: "bar",
          },
        ],
      },
    ],
    "Tank Actuators": [
      {
        topic: "mavlink/simba_rocket_heartbeat",
        fields: [
          {
            name: "main_valve",
            extract: (data) => (data.values & 0x01 ? "OPEN" : "CLOSED"),
            display: "Main Valve",
          },
          {
            name: "tank_vent",
            extract: (data) => (data.values & 0x02 ? "OPEN" : "CLOSED"),
            display: "Tank Vent",
          },
          {
            name: "dump_valve",
            extract: (data) => (data.values & 0x04 ? "OPEN" : "CLOSED"),
            display: "Dump Valve",
          }
        ],
      },
    ],
    Engine: [
      {
        topic: "mavlink/simba_rocket_heartbeat",
        fields: [
          {
            name: "engine_computer_state",
            extract: (data) => getStatusString(data.engine_computer_state),
            display: "Engine Computer State",
          }
        ],
      },
      {
        topic: "mavlink/simba_computer_temperature",
        fields: [
          {
            name: "eb_temp",
            extract: (data) => data.eb_temp,
            display: "Computer temp",
            unit: "°C"
          }
        ],
      }
    ],
  };

  const relevantTopics = topicDataMap[title]
    ? topicDataMap[title]
        .map((config) => config.topic)
        .filter((value, index, self) => self.indexOf(value) === index)
    : [];

  function processData(topicName, data) {
    const topicConfigs =
      topicDataMap[title]?.filter((config) => config.topic === topicName) || [];

    for (const config of topicConfigs) {
      for (const field of config.fields) {
        try {
          const extractedValue = field.extract(data);
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
  }
</script>

<BaseField
  {title}
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
      width: 10px;
      height: 10px;
      border-radius: 50%;
      margin-right: 8px;
    }

    :global(.rocket-telem-class .timestamp) {
      font-size: 0.7rem;
    }
    
    .field-value {
      margin-top: 0.25rem;
    }

    .field-label {
      font-size: 0.85rem;
    }

    .field-data {
      font-size: 0.85rem;
    }
  }
</style>
