<script>
  import { onMount } from "svelte";
  import { slide } from "svelte/transition";
  import { fetchConfig } from "./lib/Utils.svelte";
  import Plot from "./Plot.svelte";  // Assuming you have a Plot component

  export let host;

  let topics = [];
  let expandedMsgType = null;
  let selectedField = null;
  let error = null;
  let loading = true;

  // New array to store multiple plots
  let plots = [];

  onMount(async () => {
    try {
      topics = await fetchConfig(host);
      loading = false;
    } catch (e) {
      error = e.message;
      loading = false;
    }
  });

  function toggleExpand(msgType) {
    expandedMsgType = expandedMsgType === msgType ? null : msgType;
    selectedField = null;
  }

  function selectField(field) {
    selectedField = field;

    // Add to plots if not already present
    if (!plots.find(p => p.val_name === field.val_name && p.msg_type === field.msg_type)) {
      if (plots.length < 6) {
        plots = [...plots, field];
      } else {
        alert("Maximum of 6 plots reached");
      }
    }
  }

  // Optional: Remove a plot from the grid
  function removePlot(field) {
    plots = plots.filter(p => p !== field);
  }
</script>

{#if loading}
  <p>Loading topics...</p>
{:else if error}
  <p class="error">Error loading topics: {error}</p>
{:else}
  <div class="container">
    <nav class="msg-type-list" aria-label="Message Types">
      {#each topics as topic (topic.id)}
        <div class="msg-type-item">
          <button
            class="msg-type-button {expandedMsgType === topic.msg_type ? 'expanded' : ''}"
            on:click={() => toggleExpand(topic.msg_type)}
            aria-expanded={expandedMsgType === topic.msg_type}
            aria-controls={"fields-" + topic.id}
            aria-haspopup="true"
          >
            <span>{topic.msg_type}</span>
            <span class="arrow {expandedMsgType === topic.msg_type ? 'expanded' : ''}">▶</span>
          </button>

          {#if expandedMsgType === topic.msg_type}
            <div id={"fields-" + topic.id} class="field-list" transition:slide>
              {#each topic.msg_fields.filter(f => f.val_name !== "header") as field}
                <div
                  class="field-item"
                  on:click={() => selectField({...field, topic: topic.topic_name, msg_type: topic.msg_type})}
                  role="button"
                  tabindex="0"
                  on:keydown={(e) => e.key === "Enter" && selectField({...field, topic: topic.topic_name, msg_type: topic.msg_type})}
                >
                  {field.val_name} {field.unit ? `(${field.unit})` : ''}
                </div>
              {/each}
            </div>
          {/if}
        </div>
      {/each}
    </nav>

    <main class="plots-grid" aria-label="Plots Grid">
      {#if plots.length === 0}
        <p>Select fields from the list to plot.</p>
      {:else}
        {#each plots as field (field.val_name + field.msg_type + field.topic)}
          <div class="plot-cell">
            <button class="remove-btn" on:click={() => removePlot(field)} aria-label="Remove plot">×</button>
            <Plot
              host={host}
              topic={field.topic}
              msg_type={field.msg_type}
              field={field.val_name}
              time_range={1}
            />
          </div>
        {/each}
      {/if}
    </main>
  </div>
{/if}

<style>
  .container {
  display: flex;
  flex-direction: row;
  margin-top: calc(var(--navbar-height) - 20px);
  height: auto;
  min-height: calc(100vh - var(--navbar-height));
  }

  .msg-type-list {
      flex-shrink: 0; /* Don't let it resize when content changes */
  height: 100%; /* Lock to parent height */
    direction: rtl;
    overflow-y: auto;
    max-height: 100vh;
    width: 250px;
    background: var(--snd-bg-color);
    border-right: 1px solid #333;
    padding: 0.5rem;
    box-sizing: border-box;
    
  }

  .msg-type-list > .msg-type-item {
    direction: ltr;
  }

  .msg-type-item {
    user-select: none;
    margin-bottom: 0.3rem;
  }

  .msg-type-button {
    cursor: pointer;
    padding: 0.5rem 0.8rem;
    width: 100%;
    border: none;
    background: var(--bg-color);
    color: var(--text-color);
    border-radius: 4px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    font-weight: 600;
    font-size: 0.85rem;
  }

  .msg-type-button:hover {
    background-color: var(--nav-hover);
  }

  .msg-type-button.expanded {
    background-color: #ff965f;
    color: #181b1f;
  }

  .arrow {
    transition: transform 0.3s ease;
  }

  .arrow.expanded {
    transform: rotate(90deg);
  }

  .field-list {
    margin-top: 0.3rem;
    padding-left: 1.2rem;
    background-color: var(--snd-bg-color);
    border-radius: 0 0 4px 4px;
  }

  .field-item {
    cursor: pointer;
    padding: 0.3rem 0.5rem;
    border-bottom: 1px solid #333;
    color: var(--text-color);
    font-size: 0.9rem;
    background-color: var(--snd-bg-color);
  }

  .field-item:last-child {
    border-bottom: none;
  }

  .field-item:hover {
    background-color: var(--nav-hover);
  }

  /* New styles for the plot grid */
  .plots-grid {
    flex-grow: 1;
    height: calc(100vh - var(--navbar-height));
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    grid-template-rows: repeat(3, 1fr);
    gap: 10px;
    padding: 1rem;
    background: var(--bg-color);
    overflow-y: auto;
  }

  .plot-cell {
    position: relative;
    background: var(--snd-bg-color);
    border-radius: 6px;
    color: white;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    overflow: hidden; /* just in case */
  }

  .remove-btn {
    position: absolute;
    top: 4px;
    right: 6px;
    background: transparent;
    border: none;
    color: #ff6666;
    font-size: 1.3rem;
    cursor: pointer;
  }

  .remove-btn:hover {
    color: #ff2222;
  }
</style>
