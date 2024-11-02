<script>
  import Dashboard from './Dashboard.svelte';
  import Gradients from './lib/Gradients.svelte';
  import NavBar from './NavBar.svelte';
  import InFlight from './InFlight.svelte';
  import Cameras from './Cameras.svelte';
  import Map from './Map.svelte'
  import Grafana from './Grafana.svelte';

  let currentView = 'dashboard';
  // @ts-ignore
  const host = process.env.IP_ADDRESS;

  function handleNavigation(event) {
    currentView = event.detail;
  }

</script>

<main>
  <NavBar on:navigate={handleNavigation} {currentView} {host} />
  {#if currentView === 'dashboard'}
    <Dashboard {host}/>
  {:else if currentView === 'inflight'}
    <InFlight {host} />
  {:else if currentView === 'cameras'}
    <Cameras {host} />
  {:else if currentView === 'map'}
    <Map {host} />
  {:else if currentView === 'grafana'}
    <Grafana {host} />
  {/if}
  <Gradients />
</main>

<style>
</style>

