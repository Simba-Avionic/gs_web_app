<script>
  import { onMount } from "svelte";

  export let topic;
  let telem_data;

  const evtSource = new EventSource(
    `http://localhost:8000/${topic.topic_name}`,
  );

  evtSource.onmessage = function (event) {
    telem_data = JSON.parse(event.data);
    // console.log(telem_data);
  };
</script>

<div class="rocket-field">
  <div
    class="rocket-status-indicator {telem_data != 'None' ||
    null ||
    NaN ||
    undefined
      ? 'green-status'
      : 'red-status'}"
  ></div>
  <span class="rocket-field-text">{topic.topic_name}</span>
  {#each topic.msg_fields as field}
    {#if telem_data != undefined && field.val_name !== "header"}
      <span class="rocket-field-value">{telem_data[field.val_name]}</span>
    {/if}
  {/each}
  
</div>

<style>
  .rocket-field {
    white-space: nowrap;
    display: flex;
    justify-content: space-between;
    align-items: center;
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
