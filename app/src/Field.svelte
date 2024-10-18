<script>
  import { onMount, onDestroy } from "svelte";
  import { rosTimeToFormattedTime } from "./lib/Utils.svelte";

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
    // const host = process.env.IP_ADDRESS;
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
    if (typeof value === 'object' && value !== null) {
      return Object.keys(value).map(key => {
        return `
          <div class="${class_name}-nested-field">
            <span>${key}:</span>
            <span>${parseFloat(value[key]).toFixed(2)}</span>
          </div>
        `;
      }).join('');
    } else {
      if (field.type.includes("float")) {
        return `<span>${parseFloat(value).toFixed(2)}</span>`;
      }
      else {
        return `<span>${value}</span>`;
      }
    }
  }
</script>

<div class="{class_name}-field">
  <div
    class="status-indicator {temp !== 'None' &&
    temp !== null &&
    temp !== undefined
      ? 'green-status'
      : 'red-status'}"
  ></div>
  <div class="{class_name}-field-content">
    <span class="{class_name}-field-text">{String(topic.topic_name)}</span>
    {#if telem_data != undefined && telem_data !== "None" && telem_data !== null}
      <span class="timestamp"
        >{rosTimeToFormattedTime(
          telem_data.header.stamp.sec,
          telem_data.header.stamp.nanosec,
        )}</span
      >
      {#each topic.msg_fields as field}
        {#if field.val_name !== "header"}
          <div class="{class_name}-field-value">
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

  .gs-field:first-child {
    border-top: 1px solid #eee;
  }

  .gs-field-text {
    flex: 0.2;
    color: #fff;
  }

  .status-indicator {
    width: 16px;
    height: 16px;
    border-radius: 50%;
    margin-right: 8px;
    margin-top: 2px;
  }

  .gs-field-value {
    color: whitesmoke;
    margin-top: 8px;
  }

  .gs-field-value span:first-child {
    font-weight: bold;
    margin-right: 4px;
  }

  .gs-nested-field {
    font-weight: bold;
    padding-left: 50px;
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

  .rocket-field {
    white-space: nowrap;
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    /* align-items: center; */
    padding: 10px;
    border-bottom: 1px solid #eee; /* Add border to the bottom of each field */
    text-align: left;
  }

  .rocket-field:last-child {
    border-bottom: none; /* Remove border from the last field */
  }

  .rocket-field-text {
    flex: 1;
    color: #fff; /* Text color */
    min-width: 0; /* Allow text to overflow if needed */
    overflow: hidden; /* Hide overflow text */
    white-space: nowrap; /* Prevent text wrapping */
    text-overflow: ellipsis; /* Show ellipsis if text overflows */
  }

  .rocket-status-indicator {
    width: 16px; /* Adjust width as needed */
    height: 16px; /* Adjust height as needed */
    border-radius: 50%;
    margin-right: 10px;
  }

  .rocket-field-value {
    color: whitesmoke; /* Text color */
  }

  .rocket-field-content {
    flex: 1;
  }

  .rocket-field-value span:first-child {
    font-weight: bold;
    margin-right: 4px;
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
