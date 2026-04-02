<script>
  import BaseField from "./BaseField.svelte";
  import { getStateString } from "./Utils.svelte";

  export let title;
  export let fieldConfigs = [];

  let isExpanded = true;
  let extractedData = {};
  let hasNonOkStatus = false;

  $: relevantTopics = [...new Set(fieldConfigs.map((config) => config.topic))];

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
          finalVal = getStateString(rawVal, config.enum);
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

    // TRIGGER SVELTE REACTIVITY
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
    display: flex;
    justify-content: space-between;
    width: 100%;
    padding: 0.1rem 0;
    /* border-bottom: 1px solid rgba(255, 255, 255, 0.1); */
    border-bottom: 1px solid var(--nav-hover);
  }

  .field-value:last-child {
    border-bottom: none;
    padding-bottom: 0;
  }

  .field-value:first-child {
    padding-top: 0.5rem;
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
      font-size: 1.2rem;
    }
    :global(.rocket-telem-class .field-top-row) {
      font-size: 1.4rem;
    }
    :global(.rocket-telem-class .status-indicator) {
      margin-right: 1rem;
    }
    :global(.rocket-telem-class .timestamp) {
      font-size: 0.9rem;
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
      font-size: 0.75rem;
    }

    .field-label {
      font-size: 0.85rem;
    }

    .field-data {
      font-size: 0.85rem;
    }
  }

  /* For resolutions between 1024px and 1280px */
  @media (min-width: 1025px) {
    .field-value {
      padding: 0.25rem 0;
    }
  }

  /* For resolutions strictly higher than 1280px */
  @media (min-width: 1281px) {
    .field-value {
      padding: 0.5rem 0;
    }

    .field-value:first-child {
      padding-top: 1rem;
    }
  }
</style>
