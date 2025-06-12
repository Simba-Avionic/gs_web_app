<script>
  import { onMount, onDestroy } from "svelte";
  import { createEventDispatcher } from "svelte";

  const dispatch = createEventDispatcher();

  export let host;
  export let currentView;
  let temp;
  let telem_data;
  let socket;

  let currentTime;
  let interval;

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

  // TODO: Add ws for mavlink/simba_ack - rocket state

  function initializeWebSocket() {
    socket = new WebSocket(`ws://${host}:8000/server/telemetry`);
    

    socket.onmessage = (event) => {
      temp = JSON.parse(event.data);
      if (temp !== "None" && temp !== null && temp !== undefined) {
        telem_data = temp;
      }
      dispatch("telemetryChange", telem_data);
    };

    socket.onopen = () => {
      console.log('WebSocket connection for server/telemetry established');
    };

    socket.onclose = (event) => {
      if (!event.wasClean) {
        setTimeout(() => {
          console.log(`Reconnecting to WebSocket for server/telemetry...`);
          initializeWebSocket();
        }, 5000);
      }
    };
  }

  function reloadPage() {
    window.location.reload();
  }

  function navigate(view) {
    window.location.hash = view;
    dispatch("navigate", view);
  }

  onDestroy(() => {
    clearInterval(interval);
    closeSocket();
  });
</script>

<!-- svelte-ignore a11y-invalid-attribute -->
<nav class="navbar">
  <div class="navbar-options">
    <img src="icons/simba_logo.png" alt="Logo" class="logo" />
    <a href="#" class="{currentView === 'dashboard' ? 'active' : ''}" on:click|preventDefault={() => navigate("dashboard")}
      >Overview</a
    >
    <!-- <a href="#" class="{currentView === 'inflight' ? 'active' : ''}" on:click|preventDefault={() => navigate("inflight")}
      >Rocket</a
    > -->
    <a href="#" class="{currentView === 'map' ? 'active' : ''}" on:click|preventDefault={() => navigate("map")}
      >Map</a
    >
    <a href="#" class="{currentView === 'grafana' ? 'active' : ''}" on:click|preventDefault={() => navigate("grafana")}
      >Grafana</a
    >
    
    <!-- <a href="#" class="{currentView === 'simulation' ? 'active' : ''}" on:click|preventDefault={() => navigate("simulation")}
      >Simulation</a
    >
    <a href="#" class="{currentView === 'cameras' ? 'active' : ''}" on:click|preventDefault={() => navigate("cameras")}
      >Cameras</a
    > -->
  
  </div>
  <div class="rocket-state">
      <a href="#">ROCKET: </a>
      <h3> DISARMED </h3>
  </div>
  <div class="navbar-right">
    <div class="navbar-telemetry">
      <span>CPU: {telem_data?.cpu_usage ? `${telem_data?.cpu_usage}%` : 'N/A'}</span>
      <span>Memory: {telem_data?.memory_usage ? `${telem_data?.memory_usage}%` : 'N/A'}</span>
      <!-- <span>Disk: {telem_data?.disk_usage ? `${telem_data?.disk_usage}%` : 'N/A'}</span> -->
      <span>Temp: {telem_data?.temperature ? `${telem_data?.temperature.toFixed(2)}°C` : 'N/A'}</span>
      <!-- <span>Load (1m): {telem_data?.load_1_min.toFixed(2) ?? 'N/A'}</span> -->
      <!-- <span>Load (5m): {telem_data?.load_5_min.toFixed(2) ?? 'N/A'}</span> -->
    </div>
    <div class="navbar-time">
      {currentTime}
    </div>
    <button class="reload-button" on:click={reloadPage}>
      <img src="icons/refresh-icon.svg" alt="Reload" class="reload-icon" />
    </button>
  </div>
</nav>

<style>

  .rocket-state {
    display: flex;
    align-items: center;
    color: #ccccdc;
    gap: 10px;
  }

  .navbar {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px 20px;
    background-color: #181b1f;
    position: fixed;
    top: 0;
    left: 0;
    right: 0;
    width: 100%;
    z-index: 1000;
    box-sizing: border-box;
    overflow: hidden;
    height: var(--navbar-height);
  }

  .navbar-right {
    display: flex;
    align-items: center;
    gap: 20px;
  }

  .navbar-options {
    display: flex;
    align-items: center;
    gap: clamp(5px, 1vw, 30px);
  }

  .navbar-options a {
    text-decoration: none;
    font-weight: 600;
    cursor: pointer;
    padding: 5px 10px;
    transition: background-color 0.3s, color 0.3s;
    border-radius: 5px;
  }

  .navbar-options a:hover {
    background-color: rgba(204, 204, 220, 0.1);
  }

  .navbar-options a:active {
    background-color: rgba(204, 204, 220, 0.1);
  }

  .navbar-time {
    font-size: 1.5rem;
    font-weight: 600;
    font-family: monospace;
    display: flex;
    align-items: center;
    margin-left: auto;
    justify-content: flex-end;
    min-width: 65px;
  }

  .navbar-telemetry {
    display: flex;
    align-items: center;
    font-size: 0.7rem;
    border: 1px solid rgba(204, 204, 220, 0.25);
    border-radius: 1em;
    width: 200px;
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
    background-color:#ccccdc;;
    border: none;
    padding: 5px 10px;
    cursor: pointer;
    font-size: 1em;
    transition: background-color 0.3s;
    border-radius: 0.8rem;
  }

  .reload-button:hover {
    background-color: #777;
  }

  .reload-icon {
    width: 20px;
    height: 20px;
    padding-top: 3px;
  }

  .active {
    background-color: #555;
  }
</style>
