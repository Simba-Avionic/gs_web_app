<script>
    import { onMount } from "svelte";
    export let topic;
    let telem_data;

    const evtSource = new EventSource(
        `http://localhost:8000/${topic.topic_name}`);
    
    evtSource.onmessage = function (event) {
      telem_data = JSON.parse(event.data)
        // console.log(event);
    };

</script>

<div class="gs-field">
  <div
    class="gs-status-indicator {telem_data != 'None' || null || NaN || undefined
      ? 'green-status'
      : 'red-status'}"
  ></div>
  <span class="gs-field-text">{topic.topic_name}</span>
  <span class="gs-field-value">{telem_data}</span>
</div>

<style>
  .gs-field {
    display: flex;
    align-items: center;
    padding: 12px;
    border-bottom: 1px solid #eee; /* Add border to the bottom of each field */
    text-align: left;
  }

  .gs-field:last-child {
    border-bottom: none; /* Remove border from the last field */
  }

  .gs-field-text {
    flex: 0.2;
    color: #fff; /* Text color */
  }

  .gs-status-indicator {
    width: 16px; /* Adjust width as needed */
    height: 16px; /* Adjust height as needed */
    border-radius: 50%;
    margin-right: 16px;
  }

  .gs-field-value {
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
