<script>
  // @ts-nocheck
  import { onMount } from "svelte";
  import Dashboard from './Dashboard.svelte';
  import Gradients from './lib/Gradients.svelte';
  import NavBar from './NavBar.svelte';
  import Map from './Map.svelte'
  import Plots from './Plots.svelte';
  import Cameras from './Cameras.svelte';

  let currentView = 'dashboard';
  const ip = process.env.IP_ADDRESS;
  const port = process.env.SERVER_PORT || 8000;
  const timezone = process.env.TIMEZONE || 'UTC';

  const host = `${ip}:${port}`;

  function handleNavigation(event) {
    currentView = event.detail;
  }

  onMount(() => {

    const hash = window.location.hash.slice(1);
    if (['dashboard', 'inflight', 'map', 'plots', 'cameras'].includes(hash)) {
      currentView = hash;
    }

    window.addEventListener('hashchange', () => {
      const hash = window.location.hash.slice(1);
      if (['dashboard', 'inflight', 'map', 'plots', 'cameras'].includes(hash)) {
        currentView = hash;
      }
    });
  });

</script>

<svelte:head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
</svelte:head>

<main>
  <NavBar on:navigate={handleNavigation} {currentView} {host} {timezone} />
  {#if currentView === 'dashboard'}
    <Dashboard {host}/>
  {:else if currentView === 'plots'}
    <Plots {host}/>
  {:else if currentView === 'map'}
    <Map {host} />
  {:else if currentView === 'cameras'}
    <Cameras {host} />
  {/if}
  <Gradients />
</main>

<style>
</style>

