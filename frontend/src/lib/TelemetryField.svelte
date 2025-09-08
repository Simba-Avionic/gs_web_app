<script>
  import BaseField from "./BaseField.svelte";
  import { renderField, stripSimbaPrefix } from "./Utils.svelte";

  export let topic;
  export let host;

  let isExpanded = false;

  // For TelemetryField, we only have one topic to monitor
  const topicNames = [topic.topic_name];
</script>

<BaseField
  title={stripSimbaPrefix(topic.topic_name)}
  {host}
  {topicNames}
  bind:isExpanded
  on:telemetryChange
  className="telemetry-class"
>
  <svelte:fragment slot="default" let:telemetryData>
    <!-- Access the telemetry data passed from BaseField -->
    {#if telemetryData[topic.topic_name]}
      <div class="fields-column">
        {#each topic.msg_fields as field}
          {#if field.val_name !== "header"}
            <div class="field-value">
              <span>{field.alt_name ? field.alt_name : field.val_name}:</span>
              <span>
                {@html renderField(telemetryData[topic.topic_name], field)}
                {#if field.unit}
                  {" " + field.unit}
                {/if}
              </span>
            </div>
          {/if}
        {/each}
      </div>
    {/if}
  </svelte:fragment>
</BaseField>

<style>
  .field-value {
    white-space: normal;
    overflow-wrap: break-word;
    word-break: break-word;
    width: 100%;
  }

  .field-value span:first-child {
    font-weight: bold;
    margin-right: 4px;
  }

  :global(.telemetry-class .field-top-row) {
    padding: 12px;
    border-bottom: 1px solid var(--border-color);
    border-top: 1px solid var(--border-color);
    text-align: left;
    min-width: 0;
    width: 100%;
    box-sizing: border-box;
    border-bottom: none;
    margin: 0;
  }

  :global(.telemetry-class .telemetry-data) {
    padding: 0;
    margin: 0;
  }

  :global(.telemetry-class .fields-column) {
    padding: 0px 0px 16px 32px;
    display: flex;
    flex-direction: column;
    gap: 6px;
    width: 100%;
    box-sizing: border-box;
  }

  @media (min-width: 1920px) {
    :global(.telemetry-class .fields-column) {
      font-size: 1rem;
    }
  }
</style>
