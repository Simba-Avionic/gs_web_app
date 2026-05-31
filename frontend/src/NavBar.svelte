<script>
  import { onMount, onDestroy } from "svelte";
  import { createEventDispatcher } from "svelte";
  import { theme } from "./config/theme.js";

  let currentTheme;
  theme.subscribe((value) => (currentTheme = value));

  function toggleTheme() {
    theme.update((t) => (t === "light" ? "dark" : "light"));
  }

  const dispatch = createEventDispatcher();

  export let timezone;
  export let currentView;

  let temp;
  let telem_data;
  let socket;

  let networkSocket;
  let network_data = {};

  let currentTime;
  let interval;

  let isMenuOpen = false;

  currentTime = new Intl.DateTimeFormat("en-GB", {
    hour12: false,
    timeZone: timezone,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  }).format(new Date());

  onMount(() => {
    initializeWebSocket();
    initializeNetworkWebSocket();

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
      console.log(`WebSocket for server/telemetry closed.`);
    }
    if (networkSocket) {
      networkSocket.close();
      networkSocket = null;
      console.log(`WebSocket for network/telemetry closed.`);
    }
  }

  function initializeWebSocket() {
    socket = new WebSocket(`ws://${window.location.host}/server/telemetry`);

    socket.onmessage = (event) => {
      temp = JSON.parse(event.data);
      if (temp !== null && temp !== undefined && temp.status !== "timeout") {
        telem_data = temp;
      }
      dispatch("telemetryChange", telem_data);
    };

    socket.onopen = () => console.log("Connected to server/telemetry");

    socket.onclose = (event) => {
      if (!event.wasClean) {
        setTimeout(() => {
          console.log(`Reconnecting to WebSocket for server/telemetry...`);
          initializeWebSocket();
        }, 5000);
      }
    };
  }

  function initializeNetworkWebSocket() {
    networkSocket = new WebSocket(
      `ws://${window.location.host}/network/telemetry`,
    );

    networkSocket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data) {
          network_data = data;
        }
      } catch (err) {
        console.error("Error parsing network data", err);
      }
    };

    networkSocket.onopen = () => console.log("Connected to network/telemetry");

    networkSocket.onclose = (event) => {
      if (!event.wasClean) {
        setTimeout(() => {
          console.log(`Reconnecting to WebSocket for network/telemetry...`);
          initializeNetworkWebSocket();
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
    isMenuOpen = false; // Auto-close menu when a link is clicked
  }

  function toggleMenu() {
    isMenuOpen = !isMenuOpen;
  }

  onDestroy(() => {
    clearInterval(interval);
    closeSockets();
  });
</script>

<nav class="navbar">
  <div class="navbar-left">
    <img src="icons/simba_logo.png" alt="Logo" class="logo" />

    <button class="hamburger" on:click={toggleMenu} aria-label="Toggle menu">
      <svg viewBox="0 0 100 80" width="24" height="24" fill="var(--text-color)">
        <rect width="100" height="15" rx="8"></rect>
        <rect y="30" width="100" height="15" rx="8"></rect>
        <rect y="60" width="100" height="15" rx="8"></rect>
      </svg>
    </button>

    <span class="view-title">{currentView || "Dashboard"}</span>

    <div class="nav-links {isMenuOpen ? 'open' : ''}">
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
      <a
        href="#"
        class={currentView === "cameras" ? "active" : ""}
        on:click|preventDefault={() => navigate("cameras")}>Cameras</a
      >
      <a
        href="#"
        class={currentView === "SetAngle" ? "active" : ""}
        on:click|preventDefault={() => navigate("SetAngle")}>SetAngle</a
      >
    </div>
  </div>

  <div class="navbar-right">
    <div class="navbar-network">
      {#each Object.values(network_data) as device}
        <div class="network-device">
          <div
            class="svg-status-icon {device.is_online ? 'online' : 'offline'}"
            style="-webkit-mask-image: url('/icons/{device.icon}'); mask-image: url('/icons/{device.icon}');"
          ></div>

          <span class="tooltip-text">
            <strong>{device.name}</strong> ({device.location})<br />
            <small>{device.ip}</small>
          </span>
        </div>
      {/each}
    </div>

    {#if Object.keys(network_data).length > 0}
      <div class="nav-divider"></div>
    {/if}

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

    <button
      type="button"
      on:click={toggleTheme}
      class="toggle-container {currentTheme}"
    >
      <span class="icon">🌙</span>
      <span class="icon">☀️</span>
      <div class="circle"></div>
    </button>

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
    overflow: visible;
    height: var(--navbar-height);
  }

  .navbar a {
    color: var(--text-color);
  }

  .view-title {
    display: none;
    font-size: 1.3rem;
    font-weight: bold;
    color: var(--text-color);
    text-transform: capitalize;
    letter-spacing: 0.5px;
  }

  .navbar-left {
    display: flex;
    align-items: center;
    gap: 15px;
  }

  .navbar-right {
    display: flex;
    align-items: center;
    gap: 15px;
  }

  .nav-links {
    display: flex;
    align-items: center;
    gap: clamp(5px, 1vw, 30px);
  }

  .nav-links a {
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

  .nav-links a:hover {
    background-color: var(--nav-hover);
  }

  .nav-links a.active,
  .nav-links a.active:hover {
    background-color: #ff965f;
    color: #181b1f;
  }

  .hamburger {
    display: none;
    background: transparent;
    border: none;
    cursor: pointer;
    padding: 5px;
    outline: none;
  }

  .navbar-time {
    font-size: 1.3rem;
    font-weight: 600;
    font-family: monospace;
    display: flex;
    align-items: center;
    justify-content: flex-end;
    min-width: 65px;
  }

  .navbar-network {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .network-device {
    position: relative; /* Required to anchor the tooltip */
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer; /* Removed the question mark 'help' cursor */
  }

  .tooltip-text {
    visibility: hidden;
    opacity: 0;
    position: absolute;
    top: 150%; /* Pushes the tooltip below the icon */
    left: 50%;
    transform: translateX(-50%); /* Centers it perfectly */
    background-color: var(--snd-bg-color);
    color: var(--text-color);
    text-align: center;
    padding: 6px 10px;
    border-radius: 6px;
    border: 1px solid var(--border-color);
    font-size: 0.75rem;
    white-space: nowrap;
    z-index: 2000;
    transition:
      opacity 0.2s ease,
      visibility 0.2s ease;
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
    pointer-events: none; /* Stops the tooltip from glitching if mouse touches it */
  }

  .tooltip-text::after {
    content: "";
    position: absolute;
    bottom: 100%;
    left: 50%;
    transform: translateX(-50%);
    border-width: 5px;
    border-style: solid;
    border-color: transparent transparent var(--snd-bg-color) transparent;
  }

  .network-device:hover .tooltip-text {
    visibility: visible;
    opacity: 1;
  }

  .svg-status-icon {
    width: 16px;
    height: 16px;
    mask-size: contain;
    mask-repeat: no-repeat;
    mask-position: center;
    -webkit-mask-size: contain;
    -webkit-mask-repeat: no-repeat;
    -webkit-mask-position: center;
    transition: background-color 0.3s ease;
  }

  .svg-status-icon.online {
    background-color: #2ecc71;
    box-shadow: 0 0 6px rgba(46, 204, 113, 0.4);
  }

  .svg-status-icon.offline {
    background-color: #e74c3c;
  }

  .nav-divider {
    width: 1px;
    height: 24px;
    background-color: var(--border-color, #555);
  }

  .navbar-telemetry {
    display: flex;
    align-items: center;
    font-size: 0.7rem;
    width: 150px;
  }

  .navbar-telemetry span {
    display: inline-block;
    padding: 5px;
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
    width: 62px;
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
    border: none;
    outline: none;
    margin: 0;
    box-sizing: border-box;
  }

  .icon {
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
    background-color: var(--text-color);
    transition: left 0.3s ease;
  }

  .light .circle {
    left: 3px;
  }
  .dark .circle {
    left: 33px;
  }

  @media (max-width: 1280px) {
    .view-title {
      display: block;
    }

    .hamburger {
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .nav-links {
      display: none; /* Hide default inline layout */
      flex-direction: column;
      position: absolute;
      top: var(--navbar-height);
      left: 0;
      width: 200px;
      background-color: var(--snd-bg-color);
      padding: 10px;
      gap: 10px;
      border-bottom-right-radius: 10px;
      border: 1px solid var(--border-color);
      border-top: none;
      box-shadow: 4px 4px 10px rgba(0, 0, 0, 0.2);
      z-index: 999;
    }

    .nav-links.open {
      display: flex; /* Show dropdown when isMenuOpen is true */
    }

    .nav-links a {
      width: 100%;
      text-align: left;
      box-sizing: border-box;
    }
  }
</style>
