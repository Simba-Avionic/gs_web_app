<script>
  import { onMount, onDestroy } from "svelte";
  import { createEventDispatcher } from "svelte";
  import { theme } from './js/theme.js';

  let currentTheme;
  theme.subscribe(value => currentTheme = value);

  function toggleTheme() {
    theme.update(t => t === 'light' ? 'dark' : 'light');
  }

  const dispatch = createEventDispatcher();

  export let timezone;
  export let currentView;
  let temp;
  let telem_data;
  let socket;
  let ackSocket;

  let countdownValue = null;
  let countdownInterval;

  let currentTime;
  let interval;

  let rocketState = 1; // Default to DISARMED (1)
  let rocketStatus = 0; // Default to SIMBA_STATUS_OK (0)

  const ROCKET_STATES = {
    1: { text: "DISARMED", class: "state-disarmed" },
    2: { text: "ARMED", class: "state-armed" },
    3: { text: "IGNITION", class: "state-ignition" },
    4: { text: "ABORTED", class: "state-aborted" },
  };

  currentTime = new Intl.DateTimeFormat("en-GB", {
    hour12: false,
    timeZone: timezone,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  }).format(new Date());

  onMount(() => {
    initializeWebSocket();
    // initializeAckWebSocket();

    interval = setInterval(() => {
      currentTime = new Intl.DateTimeFormat("en-GB", {
        hour12: false,
        timeZone: timezone,
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      }).format(new Date());
    }, 1000);
  });

  function closeSockets() {
    if (socket) {
      socket.close();
      socket = null;
      console.log(
        `WebSocket for server/telemetry closed on component destroy.`,
      );
    }

    if (ackSocket) {
      ackSocket.close();
      ackSocket = null;
      console.log(`WebSocket for mavlink/ack closed.`);
    }
  }

  function initializeAckWebSocket() {
    ackSocket = new WebSocket(`ws://${window.location.host}/mavlink/simba_ack`);

    ackSocket.onmessage = (event) => {
      try {
        const ackData = JSON.parse(event.data);
        if (ackData !== "None" && ackData !== null && ackData !== undefined) {
          if (ackData.status !== undefined) {
            rocketStatus = ackData.status;
            if (rocketStatus === 0 && ackData.state !== undefined) {
              rocketState = ackData.state;

              if (rocketState == 3) {
                startCountdown();
              } else if (rocketState !== 3 && countdownValue !== null) {
                stopCountdown();
              }
            }
          }

          dispatch("rocketStateChange", {
            state: rocketState,
            status: rocketStatus,
          });
        }
      } catch (error) {
        console.error("Error parsing ACK message:", error);
      }
    };

    ackSocket.onopen = () => {
      // console.log("Connected to mavlink/ack");
    };

    ackSocket.onclose = (event) => {
      if (!event.wasClean) {
        setTimeout(() => {
          console.log(`Reconnecting to WebSocket for mavlink/ack...`);
          initializeAckWebSocket();
        }, 5000);
      }
    };
  }

  function startCountdown() {
    stopCountdown();
    countdownValue = 10;
    countdownInterval = setInterval(() => {
      countdownValue -= 1;

      if (countdownValue <= 0) {
        stopCountdown();
        countdownValue = 0; // Ensure we show zero at the end
      }
    }, 1000); // Update every second
  }

  function stopCountdown() {
    clearInterval(countdownInterval);
    countdownInterval = null;
  }

  function initializeWebSocket() {
    socket = new WebSocket(`ws://${window.location.host}/server/telemetry`);

    socket.onmessage = (event) => {
      temp = JSON.parse(event.data);
      if (temp !== "None" && temp !== null && temp !== undefined) {
        telem_data = temp;
      }
      dispatch("telemetryChange", telem_data);
    };

    socket.onopen = () => {
      // console.log("Connected to server/telemetry");
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
    closeSockets();
    stopCountdown();
  });
</script>

<!-- svelte-ignore a11y-invalid-attribute -->
<nav class="navbar">
  <div class="navbar-options">
    <img src="icons/simba_logo.png" alt="Logo" class="logo" />
    <a
      href="#"
      class={currentView === "dashboard" ? "active" : ""}
      on:click|preventDefault={() => navigate("dashboard")}>Overview</a
    >
    <a
      href="#"
      class={currentView === "plots" ? "active" : ""}
      on:click|preventDefault={() => navigate("plots")}>Plots</a
    >
    <a
      href="#"
      class={currentView === "map" ? "active" : ""}
      on:click|preventDefault={() => navigate("map")}>Map</a
    >
    <!-- <a href="#" class="{currentView === 'simulation' ? 'active' : ''}" on:click|preventDefault={() => navigate("simulation")}
      >Simulation</a
    >  -->
    <a href="#" class="{currentView === 'cameras' ? 'active' : ''}" on:click|preventDefault={() => navigate("cameras")}
      >Cameras</a
    >
  </div>
  <!-- <div class="rocket-state">
    <a href="#">ROCKET: </a>
    <h3 class={ROCKET_STATES[rocketState]?.class || "state-unknown"}>
      {ROCKET_STATES[rocketState]?.text || "UNKNOWN"}
    </h3>
    {#if countdownValue !== null}
      <div class="countdown">
        <span class="countdown-value">{countdownValue}</span>
      </div>
    {/if}
  </div> -->
  <div class="navbar-right">
    <div class="navbar-telemetry">
      <span
        >CPU: {telem_data?.cpu_usage
          ? `${telem_data?.cpu_usage}%`
          : "N/A"}</span
      >
      <span
        >Memory: {telem_data?.memory_usage
          ? `${telem_data?.memory_usage}%`
          : "N/A"}</span
      >
      <span
        >Temp: {telem_data?.cpu_temperature
          ? `${telem_data?.cpu_temperature.toFixed(1)}°C`
          : "N/A"}</span
      >
    </div>
    <div class="navbar-time">
      {currentTime}
    </div>
    <div on:click={toggleTheme} class="toggle-container {currentTheme}">
      <span class="icon">🌙</span>
      <span class="icon">☀️</span>
      <div class="circle"></div>
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
    color: var(--text-color);
    gap: 10px;
  }

  .navbar {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px 20px;
    background-color: var(--snd-bg-color);
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

  .navbar a {
     color: var(--text-color);
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
    font-size: 1.2rem;
    text-decoration: none;
    font-weight: 600;
    cursor: pointer;
    padding: 5px 10px;
    transition:
      background-color 0.3s,
      color 0.3s;
    border-radius: 5px;
    color: var(--text-color);
  }

  .navbar-options a:hover {
    background-color: var(--nav-hover);
  }

  .navbar-options .active {
    background-color: #ff965f;
    color: #181b1f;
  }

  .navbar-time {
    font-size: 1.3rem;
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
    /* border: 0.05rem solid var(--border-color); */
    /* border-radius: 1em; */
    width: 150px;
  }

  .navbar-telemetry span {
    display: inline-block;
    padding: 10px;
  }

  .logo {
    width: 40px;
    height: auto;
  }

  .reload-button {
    background-color: #ccccdc;
    border: none;
    padding: 5px 10px;
    cursor: pointer;
    font-size: 1em;
    transition: background-color 0.3s;
    border-radius: 0.8rem;
  }

  .reload-button:hover {
    background-color: var(--selection-color);
  }

  .reload-icon {
    width: 20px;
    height: 20px;
    padding-top: 3px;
  }

  .toggle-container {
    width: 50px;
    height: 30px;
    background-color: var(--bg-color, #333);
    border-radius: 9999px;
    position: relative;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 0 6px;
    transition: background-color 0.5s ease;
  }

  .theme-icon {
    width: 14px;
    height: 14px;
    pointer-events: none;
  }

  .circle {
    position: absolute;
    top: 3px;
    width: 24px;
    height: 24px;
    border-radius: 50%;
    background-color: white;
    transition: left 0.3s ease;
  }

  .light .circle {
    left: 3px;
  }

  .dark .circle {
    left: 33px;
  }

  .active {
    background-color: var(--selection-color);
  }

  .state-disarmed {
    color: var(--text-color);
  }

  .state-armed {
    font-weight: bold;

    background: linear-gradient(45deg, #fa6400, #ff9830, #ff7809, #feb356);
    -webkit-background-clip: text;
    background-clip: text;
    -webkit-text-fill-color: transparent;
    animation: gradientAnimation 1s ease infinite;
  }

  .state-ignition {
    font-weight: bold;

    background: linear-gradient(45deg, #388729, #5aa54b, #99d88d, #caf2c2);
    -webkit-background-clip: text;
    background-clip: text;
    -webkit-text-fill-color: transparent;
    animation: gradientAnimation 1s ease infinite;
  }

  .state-aborted {
    font-weight: bold;

    background: linear-gradient(45deg, #c41934, #de304d, #f24865);
    -webkit-background-clip: text;
    background-clip: text;
    -webkit-text-fill-color: transparent;
    animation: gradientAnimation 1s ease infinite;
  }

  .state-unknown {
    color: #999999;
  }

  @keyframes gradientAnimation {
    0% {
      background-position: 0% 50%;
    }
    100% {
      background-position: 200% 50%;
    }
  }
</style>
