<script>
  import { onMount } from "svelte";
  import { createEventDispatcher } from "svelte";

  export let currentView;

  let currentTime = new Intl.DateTimeFormat("en-GB", {
    hour12: false,
    timeZone: "UTC",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  }).format(new Date());

  // Function to update the current time every second in UTC
  onMount(() => {
    const interval = setInterval(() => {
      currentTime = new Intl.DateTimeFormat("en-GB", {
        hour12: false,
        timeZone: "UTC",
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      }).format(new Date());
    }, 1000);

    // Clear interval when the component is destroyed
    return () => clearInterval(interval);
  });

  // Function to reload the page
  function reloadPage() {
    window.location.reload();
  }

  const dispatch = createEventDispatcher();

  function navigate(view) {
    dispatch("navigate", view);
  }
</script>

<nav class="navbar">
  <div class="navbar-options">
    <!-- Logo placeholder -->
    <img src="src/assets/simba_logo.png" alt="Logo" class="logo" />
    <a href="#" class="{currentView === 'inflight' ? 'active' : ''}" on:click|preventDefault={() => navigate("dashboard")}
      >Pre-Flight</a
    >
    <a href="#" class="{currentView === 'inflight' ? 'active' : ''}" on:click|preventDefault={() => navigate("inflight")}
      >In-Flight</a
    >
  </div>
  <div class="navbar-time">
    {currentTime}
    <button class="reload-button" on:click={reloadPage}>
      <img src="src/assets/refresh-icon.svg" alt="Reload" class="reload-icon" />
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
    position: fixed; /* Make the navbar fixed to the top */
    top: 0; /* Position at the top of the screen */
    left: 0; /* Align the navbar to the left of the screen */
    right: 0; /* Align the navbar to the right of the screen */
    width: 100%; /* Ensure it takes the full width of the screen */
    z-index: 1000; /* Ensure it stays on top of other content */
    box-sizing: border-box; /* Include padding and border in the element's total width */
    overflow: hidden; /* Prevent content from overflowing */
  }

  .navbar-options {
    display: flex;
    align-items: center;
    gap: 20px;
  }

  .navbar-options a {
    text-decoration: none;
    color: #fff;
    font-weight: bold;
    cursor: pointer;
    padding: 5px 10px;
    transition:
      background-color 0.3s,
      color 0.3s; /* Smooth transition for hover effect */
  }

  .navbar-options a:hover {
    background-color: #555; /* Change background on hover */
    color: #fff; /* Keep the text color white */
  }

  .navbar-options a:active {
    background-color: #777; /* Change background on click */
    color: #fff; /* Keep the text color white */
  }

  .navbar-time {
    font-size: 16px;
    display: flex;
    align-items: center;
    gap: 20px; /* Space between time and reload button */
  }

  .logo {
    width: 40px; /* Adjust the size of the logo */
    height: auto;
  }

  .reload-button {
    background-color: #fff;
    color: #fff;
    border: none;
    padding: 5px 10px;
    cursor: pointer;
    font-size: 14px;
    transition: background-color 0.3s; /* Smooth transition for hover effect */
  }

  .reload-button:hover {
    background-color: #777; /* Change background on hover */
  }

  .reload-icon {
    width: 20px; /* Adjust size of icon */
    height: 20px; /* Adjust size of icon */
    padding-top: 3px;
    color: #fff;
  }

  .active {
    background-color: #555; /* Highlight color for active view */
    color: #fff;
  }

  body {
    margin: 0; /* Remove default margin */
    padding-top: 50px; /* Add padding equal to navbar height to prevent content from hiding under the navbar */
    overflow-x: hidden; /* Prevent horizontal scrolling */
  }
</style>
