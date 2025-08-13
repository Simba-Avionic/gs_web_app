<script>
  import { onMount } from "svelte";
  import Dashboard from './Dashboard.svelte';
  import Gradients from './lib/Gradients.svelte';
  import NavBar from './NavBar.svelte';
  import Map from './Map.svelte'
  // import Plot from './Plot.svelte';
  import Plots from './Plots.svelte';


  let currentView = 'dashboard';
  // @ts-ignore
  const host = process.env.IP_ADDRESS;

  function handleNavigation(event) {
    currentView = event.detail;
  }

  onMount(() => {

    const hash = window.location.hash.slice(1);
    if (['dashboard', 'inflight', 'map', 'plots'].includes(hash)) {
      currentView = hash;
    }

    window.addEventListener('hashchange', () => {
      const hash = window.location.hash.slice(1);
      if (['dashboard', 'inflight', 'map', 'plots'].includes(hash)) {
        currentView = hash;
      }
    });
  });

</script>

<svelte:head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
</svelte:head>

<main>
  <NavBar on:navigate={handleNavigation} {currentView} {host} />
  {#if currentView === 'dashboard'}
    <Dashboard {host}/>
   <!-- {:else if currentView === 'cameras'} -->
  <!-- //  <Cameras {host} /> -->
  {:else if currentView === 'map'}
    <Map {host} />
  {:else if currentView === 'plots'}
    <Plots {host}/>
  {/if}
  <Gradients />
</main>

<style>
</style>

