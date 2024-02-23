<script>
    import { onMount } from 'svelte';
    
    let latestNumber = null;
  
    async function fetchLatestNumber() {
      try {
        const response = await fetch('http://localhost:8000/latest-number');
        const data = await response.json();
        latestNumber = data.latest_number;
      } catch (error) {
        console.error('Error fetching latest number:', error);
      }
    }
   
    async function pollLatestNumber() {
      while (true) {
        await fetchLatestNumber();
        await new Promise(resolve => setTimeout(resolve, 1000));
      }
    }
  
    onMount(pollLatestNumber);
</script>

<div>
    {#if latestNumber !== null}
        <h3>Latest Number: {latestNumber}</h3>
    {:else}
        <h3>Loading...</h3>
    {/if}
</div>
  