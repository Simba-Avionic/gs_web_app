<script>
  import BaseField from "./BaseField.svelte";
  import { getStateString } from "../../Utils.svelte";

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

        if (config.format === "matrix" && config.enumDef) {
          let bitMatrix = { EC: [], FC: [] };

          const enumValuesDict = config.enumDef.values
            ? config.enumDef.values
            : config.enumDef;

          let enumArray = Object.entries(enumValuesDict).map(([key, val]) => {
            return { name: String(val), value: Number(key) };
          });

          enumArray.forEach((entry) => {
            const val = entry.value;
            const isActive = (rawVal & val) === val;

            let cleanName = entry.name.replace(/^SIMBA_ROCKET_SERVICE_/, "");
            let cat = "";

            if (cleanName.startsWith("EC_")) {
              cat = "EC";
              cleanName = cleanName.substring(3);
            } else if (cleanName.startsWith("FC_")) {
              cat = "FC";
              cleanName = cleanName.substring(3);
            }

            bitMatrix[cat].push({ name: cleanName, isActive });
          });

          extractedData[config.label] = {
            type: "matrix",
            display: config.label,
            matrix: bitMatrix,
          };
          continue;
        }

        let finalVal = rawVal;

        if (config.mask !== undefined) {
          finalVal = rawVal & config.mask ? config.onLabel : config.offLabel;
        } else if (config.format === "status") {
          finalVal = getStateString(rawVal, config.enum);
        }

        extractedData[config.label] = {
          type: "text",
          value: finalVal,
          display: config.label,
          unit: config.unit || "",
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
      {#if fieldInfo.type === "matrix"}
        <div class="matrix-container">
          <span class="field-label matrix-title">{fieldInfo.display}:</span>
          <div class="matrix-grid">
            {#each ["EC", "FC"] as rowLabel}
              {#if fieldInfo.matrix[rowLabel] && fieldInfo.matrix[rowLabel].length > 0}
                <div class="matrix-row">
                  <span class="row-label">{rowLabel}</span>
                  <div class="led-group">
                    {#each fieldInfo.matrix[rowLabel] as dot}
                      <div class="led-wrapper" data-name={dot.name}>
                        <div
                          class="status-indicator {dot.isActive
                            ? 'green-status'
                            : 'red-status'}"
                        ></div>
                      </div>
                    {/each}
                  </div>
                </div>
              {/if}
            {/each}
          </div>
        </div>
      {:else}
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
      {/if}
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

  .matrix-container {
    display: flex;
    flex-direction: row;
    justify-content: space-between;
    align-items: flex-start;
    width: 100%;
    padding: 0.3rem 0;
    border-bottom: 1px solid var(--nav-hover);
  }

  .matrix-container:last-child {
    border-bottom: none;
  }

  .matrix-title {
    margin-bottom: 0;
    margin-top: 0.1rem;
  }

  .matrix-grid {
    display: flex;
    flex-direction: column;
    align-items: flex-end;
  }

  .matrix-row {
    display: flex;
    align-items: center;
    justify-content: flex-end;
    margin-bottom: 0.5rem;
  }

  .matrix-row:last-child {
    margin-bottom: 0;
  }

  .row-label {
    width: auto;
    font-size: 0.75rem;
    font-weight: bold;
    color: var(--text-color);
    text-align: right;
    margin-right: 0.5rem;
  }

  .led-group {
    display: flex;
    gap: 0.8rem;
    flex-wrap: wrap;
    justify-content: flex-end;
  }

  .led-wrapper {
    position: relative;
    display: inline-flex;
    cursor: help;
  }

  .led-wrapper:hover::after {
    content: attr(data-name);
    position: absolute;
    bottom: 150%;
    left: 50%;
    transform: translateX(-50%);
    background-color: var(--snd-bg-color);
    color: #ffffff;
    border: 1px solid var(--border-color);
    padding: 4px 8px;
    font-size: 0.75rem;
    font-weight: normal;
    border-radius: 4px;
    white-space: nowrap;
    z-index: 1000;
    pointer-events: none;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
  }

  .led-wrapper:hover::before {
    content: "";
    position: absolute;
    bottom: 100%;
    left: 50%;
    transform: translateX(-50%);
    border-width: 5px;
    border-style: solid;
    border-color: var(--border-color, #444) transparent transparent transparent;
    z-index: 1000;
    pointer-events: none;
  }

  .matrix-container .status-indicator {
    cursor: help;
    transition:
      background-color 0.2s ease,
      box-shadow 0.2s ease;
    margin-right: 0 !important;
    display: inline-block;
    flex-shrink: 0;
    width: 14px !important;
    height: 14px !important;
  }

  :global(.rocket-telem-class .timestamp) {
    font-size: 0.8rem;
  }

  :global(.rocket-telem-class .status-indicator) {
    width: 12px;
    height: 12px;
    border-radius: 50%;
  }

  /* Ekrany powyżej FullHD / Ultra-wide */
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

    .matrix-row {
      margin-bottom: 0.8rem;
    }
    .led-group {
      gap: 1.5rem;
    }
    .row-label {
      font-size: 0.9rem;
      margin-right: 1rem;
    }
    .matrix-container .status-indicator {
      width: 18px !important;
      height: 18px !important;
    }
  }

  /* Standardowe Laptopy */
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

    .matrix-row {
      margin-bottom: 0.3rem;
    }
    .led-group {
      gap: 0.4rem;
    }
    .row-label {
      margin-right: 6px;
    }
    .matrix-container .status-indicator {
      width: 10px !important;
      height: 10px !important;
    }
  }

  @media (min-width: 1025px) {
    .field-value {
      padding: 0.25rem 0;
    }
  }

  @media (min-width: 1281px) {
    .field-value {
      padding: 0.5rem 0;
    }
    .field-value:first-child {
      padding-top: 1rem;
    }
  }
</style>