<script>
  import { onMount, onDestroy } from "svelte";
  import { createEventDispatcher } from "svelte";

  const dispatch = createEventDispatcher();

  export let currentView;
  let temp;
  let telem_data;
  let socket;

  let currentTime;
  let interval;
  let connectionCount = 0; // Track the number of WebSocket connections

  currentTime = new Intl.DateTimeFormat("en-GB", {
    hour12: false,
    timeZone: "UTC",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  }).format(new Date());

  onMount(() => {

    initializeWebSocket();

    interval = setInterval(() => {
      currentTime = new Intl.DateTimeFormat("en-GB", {
        hour12: false,
        timeZone: "UTC",
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      }).format(new Date());
    }, 1000);

    return () => {
      clearInterval(interval);
      closeSocket();
    }
  });

  function closeSocket() {
    if (socket) {
      socket.close();
      socket = null;
      console.log(`WebSocket for server/telemetry closed on component destroy.`);
    }
  }

  function initializeWebSocket() {
    // @ts-ignore
    const host = process.env.IP_ADDRESS;
    socket = new WebSocket(`ws://${host}:8000/server/telemetry`);

    connectionCount++;
    console.log(`Initializing WebSocket connection #${connectionCount}`);

    // Define WebSocket event handlers
    socket.onmessage = (event) => {
      temp = JSON.parse(event.data);
      if (temp !== "None" && temp !== null && temp !== undefined) {
        telem_data = temp;
      }
      dispatch("telemetryChange", telem_data);
    };

    socket.onopen = () => {
      console.log(`WebSocket connection for server/telemetry established`);
    };

    socket.onerror = (error) => {
      console.error("WebSocket error:", error);
    };

    socket.onclose = (event) => {
      console.log(`WebSocket connection closed: ${event.reason}`);

      // Optional: Attempt to reconnect after a delay if the component is still mounted
      if (!event.wasClean) {
        setTimeout(() => {
          console.log(`Reconnecting to WebSocket for server/telemetry...`);
          initializeWebSocket();
        }, 3000); // Retry after 3 seconds
      }
    };
  }

  window.addEventListener('beforeunload', () => {
      closeSocket();
    });

  function reloadPage() {
    window.location.reload();
  }

  function navigate(view) {
    dispatch("navigate", view);
  }

  onDestroy(() => {
    clearInterval(interval);
    closeSocket();
  });
</script>

<nav class="navbar">
  <div class="navbar-options">
    <!-- Logo placeholder -->
    <img src="icons/simba_logo.png" alt="Logo" class="logo" />
    <a href="#" class="{currentView === 'dashboard' ? 'active' : ''}" on:click|preventDefault={() => navigate("dashboard")}
      >Pre-Flight</a
    >
    <a href="#" class="{currentView === 'inflight' ? 'active' : ''}" on:click|preventDefault={() => navigate("inflight")}
      >In-Flight</a
    >
    <a href="#" class="{currentView === 'cameras' ? 'active' : ''}" on:click|preventDefault={() => navigate("cameras")}
      >Cameras</a
    >
  </div>
  <div class="navbar-telemetry">
    <span>CPU: {telem_data?.cpu_usage ? `${telem_data?.cpu_usage}%` : 'N/A'}</span>
    <span>Memory: {telem_data?.memory_usage ? `${telem_data?.memory_usage}%` : 'N/A'}</span>
    <span>Disk: {telem_data?.disk_usage ? `${telem_data?.disk_usage}%` : 'N/A'}</span>
    <span>Temp: {telem_data?.temperature ? `${telem_data?.temperature.toFixed(2)}°C` : 'N/A'}</span>
    <span>Load (1m): {telem_data?.load_1_min.toFixed(2) ?? 'N/A'}</span>
    <span>Load (5m): {telem_data?.load_5_min.toFixed(2) ?? 'N/A'}</span>
  </div>
  <div class="navbar-time">
    {currentTime}
    <button class="reload-button" on:click={reloadPage}>
      <img src="icons/refresh-icon.svg" alt="Reload" class="reload-icon" />
    </button>
  </div>
</nav>

<style>
  .navbar {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px 20px;
    background-color: #333;
    color: #fff;
    position: fixed;
    top: 0;
    left: 0;
    right: 0;
    width: 100%;
    z-index: 1000;
    box-sizing: border-box;
    overflow: hidden;
  }

  .navbar-options {
    display: flex;
    align-items: center;
    gap: 40px;
  }

  .navbar-options a {
    text-decoration: none;
    color: #fff;
    font-weight: bold;
    cursor: pointer;
    padding: 5px 10px;
    transition: background-color 0.3s, color 0.3s;
    border-radius: 5px;
  }

  .navbar-options a:hover {
    background-color: #555;
    color: #fff;
  }

  .navbar-options a:active {
    background-color: #777;
    color: #fff;
  }

  .navbar-time {
    font-size: 16px;
    display: flex;
    align-items: center;
    gap: 20px;
  }

  .navbar-telemetry {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 9px;
    border: 1px solid #aaa;
    border-radius: 1vw;
    color: whitesmoke;
  }

  .navbar-telemetry span {
    display: inline-block;
    padding: 10px 8px;
  }

  .logo {
    width: 40px;
    height: auto;
  }

  .reload-button {
    background-color: #fff;
    color: #fff;
    border: none;
    padding: 5px 10px;
    cursor: pointer;
    font-size: 14px;
    transition: background-color 0.3s;
  }

  .reload-button:hover {
    background-color: #777;
  }

  .reload-icon {
    width: 20px;
    height: 20px;
    padding-top: 3px;
    color: #fff;
  }

  .active {
    background-color: #555;
    color: #fff;
  }

  body {
    margin: 0;
    padding-top: 50px;
    overflow-x: hidden;
  }
</style>
