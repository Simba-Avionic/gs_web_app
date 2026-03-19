<script>
  import BaseField from "./BaseField.svelte";
  import { getStatusString } from "./Utils.svelte";

  export let title;
  export let fieldConfigs = [];

  let isExpanded = true;
  let extractedData = {};
  let hasNonOkStatus = false;

  $: relevantTopics = [...new Set(fieldConfigs.map((config) => config.topic))];

  // NEW: Pre-populate the UI with "---" so the fields appear before data arrives
  // $: if (fieldConfigs.length > 0 && Object.keys(extractedData).length === 0) {
  //   let initialData = {};
  //   fieldConfigs.forEach((config) => {
  //     initialData[config.label] = {
  //       value: "---",
  //       display: config.label,
  //       unit: config.unit || "",
  //     };
  //   });
  //   extractedData = initialData;
  // }

  function processData(topicName, data) {
    const topicConfigs = fieldConfigs.filter((c) => c.topic === topicName);

    for (const config of topicConfigs) {
      try {
        const rawVal = data[config.fieldName];
        if (rawVal === undefined) continue;

        let finalVal = rawVal;

        if (config.mask !== undefined) {
          finalVal = rawVal & config.mask ? config.onLabel : config.offLabel;
        } else if (config.format === "status") {
          finalVal = getStatusString(rawVal);
        }

        extractedData[config.label] = {
          value: finalVal,
          display: config.label,
          unit: config.unit,
        };
      } catch (e) {
        console.error(`Error processing ${config.label} from ${topicName}:`, e);
      }
    }

    // TRIGGER SVELTE REACTIVITY HERE
    extractedData = { ...extractedData };
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
    {#each Object.entries(extractedData) as [label, fieldInfo]}
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
