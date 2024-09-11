<script>
  import Dashboard from './Dashboard.svelte';
  import Gradients from './Gradients.svelte';
  import NavBar from './NavBar.svelte';
  import InFlight from './InFlight.svelte';

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
  {/if}
  <Gradients />
</main>

<style>
</style>
