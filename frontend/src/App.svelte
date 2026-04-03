<script>
  // @ts-nocheck
  import { onMount } from "svelte";
  import { loadEnums } from "./stores/enums.js";
  import { VIEWS } from "./config/constants.js";

  import NavBar from "./NavBar.svelte";
  import Map from "./pages/Map.svelte";
  import Plots from "./pages/Plots.svelte";
  import Cameras from "./pages/Cameras.svelte";
  import Dashboard from "./pages/Dashboard.svelte";
  import Gradients from "./components/common/Gradients.svelte";

  let currentView = "dashboard";
  const ip = process.env.IP_ADDRESS;
  const port = process.env.SERVER_PORT || 2137;
  const timezone = import.meta.env.VITE_TIMEZONE;

  function handleNavigation(event) {
    currentView = event.detail;
  }

  onMount(() => {
    loadEnums();

    const hash = window.location.hash.slice(1);
    if (
      [
        VIEWS.DASHBOARD,
        VIEWS.INFLIGHT,
        VIEWS.MAP,
        VIEWS.PLOTS,
        VIEWS.CAMERAS,
      ].includes(hash)
    ) {
      currentView = hash;
    }

    window.addEventListener("hashchange", () => {
      const hash = window.location.hash.slice(1);
      if (
        [
          VIEWS.DASHBOARD,
          VIEWS.INFLIGHT,
          VIEWS.MAP,
          VIEWS.PLOTS,
          VIEWS.CAMERAS,
        ].includes(hash)
      ) {
        currentView = hash;
      }
    });
  });
</script>

<svelte:head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
</svelte:head>

<main>
  <NavBar on:navigate={handleNavigation} {currentView} {timezone} />
  {#if currentView === VIEWS.DASHBOARD}
    <Dashboard />
  {:else if currentView === VIEWS.PLOTS}
    <Plots />
  {:else if currentView === VIEWS.MAP}
    <Map />
  {:else if currentView === VIEWS.CAMERAS}
    <Cameras />
  {/if}
  <Gradients />
</main>

<style>
</style>
