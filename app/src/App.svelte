<script>
  import Dashboard from './Dashboard.svelte';
  import Gradients from './lib/Gradients.svelte';
  import NavBar from './NavBar.svelte';
  import InFlight from './InFlight.svelte';
  import Cameras from './Cameras.svelte';
  import Map from './Map.svelte'

  let currentView = 'dashboard';

  function handleNavigation(event) {
    currentView = event.detail;
  }

</script>

<main>
  <NavBar on:navigate={handleNavigation} {currentView} />
  {#if currentView === 'dashboard'}
    <Dashboard />
  {:else if currentView === 'inflight'}
    <InFlight />
  {:else if currentView === 'cameras'}
    <Cameras />
  {:else if currentView === 'map'}
    <Map />
  {/if}
  <Gradients />
</main>

<style>
  main {
    width: 90vw;
  }
</style>

