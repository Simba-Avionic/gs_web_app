<script>
  import { onMount, onDestroy } from "svelte";
  import { rosTimeToFormattedTime } from "./Utils.svelte";

  export let topic;
  let telem_data;
  let evtSource;

  onMount(() => {
    evtSource = new EventSource(
      `http://localhost:8000/${topic.topic_name}`
    );

    evtSource.onmessage = function (event) {
      telem_data = JSON.parse(event.data);
      if (topic.topic_name === "radio_433/telemetry" ||   
          topic.topic_name === "tanking/commands")
      {
        console.log(telem_data);
      }};

    evtSource.onerror = (error) => {
      console.error('GS EventSource failed:', error);
    };

    return () => {
      evtSource.close();
    };
  });

onDestroy(() => {
    if (evtSource) {
      evtSource.close();
}});

</script>

<div class="gs-field">
  <div
    class="gs-status-indicator {
      telem_data !== 'None' && 
      telem_data !== null && 
      telem_data !==  undefined
      ? 'green-status'
      : 'red-status'}"
  ></div>
  <div class="gs-field-content">
    <span class="gs-field-text">{String(topic.topic_name)}</span>
    {#if telem_data != undefined}
      <span class="timestamp">{rosTimeToFormattedTime(
          telem_data.header.stamp.sec,
          telem_data.header.stamp.nanosec
        )}</span>
      {#each topic.msg_fields as field}
        {#if field.val_name !== "header"}
          <div class="gs-field-value">
            <span>{field.val_name}:</span>
            {#if field.type.includes("float")}
              <span>{parseFloat(telem_data[field.val_name].toFixed(2)) + " " + field.unit}</span>
            {:else}
              <span>{telem_data[field.val_name] + " " + field.unit}</span>
            {/if}
          </div>
        {/if}
      {/each}
    {/if}
  </div>
</div>

<style>
  .gs-field {
    display: flex;
    align-items: flex-start;
    padding: 12px;
    border-bottom: 1px solid #eee;
    text-align: left;
  }

  .gs-field-content {
    flex: 1;
  }

  .gs-field:last-child {
    border-bottom: none;
  }

  .gs-field-text {
    flex: 0.2;
    color: #fff;
  }

  .gs-status-indicator {
    width: 16px;
    height: 16px;
    border-radius: 50%;
    margin-right: 16px;
  }

  .gs-field-value {
    color: whitesmoke;
    margin-top: 8px;
  }

  .gs-field-value span:first-child {
    font-weight: bold;
    margin-right: 4px;
  }

  .timestamp {
    margin-left: 8px; /* Add spacing between topic name and timestamp */
    color: #aaa; /* Adjust timestamp color */
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
