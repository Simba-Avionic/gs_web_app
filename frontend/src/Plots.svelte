<script>
  import { onMount, onDestroy } from "svelte";
  import { slide } from "svelte/transition";
  import { fetchConfig } from "./lib/Utils.svelte";
  import Plot from "./Plot.svelte";
  import { fade } from "svelte/transition";
  import { fetchMessageDefs } from "./lib/Utils.svelte";

  export let host;

  let sockets = [];
  let topics = [];
  let msg_defs = [];
  let plots = [];
  let expandedMsgType = null;
  let expandedNestedField = null;
  let selectedField = null;
  let error = null;
  let loading = true;

  function getNestedFields(msgDef) {
    const def = msg_defs?.find((d) => d.msg_type === msgDef);
    return def ? def.msg_fields.filter((f) => f.val_name !== "header") : [];
  }

  function toggleNestedField(field) {
    expandedNestedField = expandedNestedField === field ? null : field;
  }

  function updateTopicStatus(name, status) {
    topics = topics.map((t) => (t.topic_name === name ? { ...t, status } : t));
  }

  onMount(async () => {
    try {
      const savedPlots = localStorage.getItem("plots");
      if (savedPlots) {
        plots = JSON.parse(savedPlots);
      }

      topics = (await fetchConfig(host)).map((t) => ({
        ...t,
        status: "red",
        lastSeen: 0,
      }));

      msg_defs = await fetchMessageDefs(host);

      console.log(msg_defs);

      topics.forEach((topic) => {
        const ws = new WebSocket(`ws://${host}/${topic.topic_name}`);
        sockets.push(ws);

        ws.onmessage = (event) => {
          const data = JSON.parse(event.data);
          if (data !== "None" && data !== null && data !== undefined) {
            updateTopicStatus(topic.topic_name, "green");
          } else {
            updateTopicStatus(topic.topic_name, "red");
          }
        };

        ws.onclose = () => updateTopicStatus(topic.topic_name, "red");
        ws.onerror = (err) => updateTopicStatus(topic.topic_name, "red");
      });

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

  function savePlots() {
    localStorage.setItem("plots", JSON.stringify(plots));
  }

  function selectField(field) {
    selectedField = field;

    if (
      !plots.find(
        (p) =>
          p.val_name === field.val_name &&
          p.msg_type === field.msg_type &&
          p.topic === field.topic,
      )
    ) {
      if (plots.length < 6) {
        plots = [...plots, field];
        savePlots();
      } else {
        alert(`Maximum of 6 plots reached`);
      }
    }
  }

  function removePlot(field) {
    plots = plots.filter((p) => p !== field);
    console.log("Removed plot:", field);
    console.log(plots);
    savePlots();
  }

  onDestroy(() => {
    sockets.forEach((ws) => ws.close());
  });
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
            class="msg-type-button {expandedMsgType === topic.msg_type
              ? 'expanded'
              : ''}"
            on:click={() => toggleExpand(topic.msg_type)}
            aria-expanded={expandedMsgType === topic.msg_type}
            aria-controls={"fields-" + topic.id}
            aria-haspopup="true"
          >
            <div class="msg-type-label">
              <div
                class="status-indicator {topic.status === 'green'
                  ? 'green-status'
                  : 'red-status'}"
              ></div>
              <span>{topic.msg_type}</span>
            </div>

            <span
              class="arrow {expandedMsgType === topic.msg_type
                ? 'expanded'
                : ''}">▶</span
            >
          </button>

          {#if expandedMsgType === topic.msg_type}
            <div id={"fields-" + topic.id} class="field-list" transition:slide>
              {#each topic.msg_fields.filter((f) => f.val_name !== "header") as field}
                <div class="field-item-wrapper">
                  <div
                    class="field-item"
                    on:click={() =>
                      field.msg_def
                        ? toggleNestedField(field)
                        : selectField({
                            ...field,
                            topic: topic.topic_name,
                            msg_type: topic.msg_type,
                          })}
                    role="button"
                    tabindex="0"
                    on:keydown={(e) =>
                      e.key === "Enter" &&
                      (field.msg_def
                        ? toggleNestedField(field)
                        : selectField({
                            ...field,
                            topic: topic.topic_name,
                            msg_type: topic.msg_type,
                          }))}
                  >
                    <span
                      >{field.alt_name ?? field.val_name}
                      {field.unit ? `(${field.unit})` : ""}</span
                    >

                    {#if field.msg_def}
                      <span
                        class="arrow {expandedNestedField === field
                          ? 'expanded'
                          : ''}"
                      >
                        ▶
                      </span>
                    {/if}
                  </div>

                  {#if expandedNestedField === field && field.msg_def}
                    <div class="nested-field-list" transition:slide>
                      {#each getNestedFields(field.msg_def) as nestedField}
                        <!-- svelte-ignore a11y-click-events-have-key-events -->
                        <!-- svelte-ignore a11y-no-static-element-interactions -->
                        <div
                          class="field-item nested"
                          on:click={() =>
                            selectField({
                              ...nestedField,
                              topic: topic.topic_name,
                              msg_type: topic.msg_type,
                              parent: field.val_name,
                            })}
                        >
                          <span
                            >{nestedField.alt_name ?? nestedField.val_name}
                            {nestedField.unit
                              ? `(${nestedField.unit})`
                              : ""}</span
                          >
                        </div>
                      {/each}
                    </div>
                  {/if}
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
        {#each plots as field, i}
          <div out:fade={{ duration: 200 }}>
            <Plot
              {host}
              {field}
              time_range={1}
              onRemove={() => removePlot(field)}
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
    margin-top: calc(var(--navbar-height));
    min-height: calc(100vh - var(--navbar-height));
  }

  .field-item.nested {
    padding-left: 1rem;
    font-size: 0.85rem;
  }

  .status-indicator {
    width: 10px;
    height: 10px;
    border-radius: 50%;
  }

  .msg-type-list {
    flex-shrink: 0;
    direction: rtl;
    overflow-y: auto;
    max-height: 100vh;
    width: 250px;
    background: var(--snd-bg-color);
    border-right: 1px solid #333;
    padding: 0.5rem;
    box-sizing: border-box;
  }

  .plots-grid:has(p) {
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 1.2rem;
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

  .msg-type-label {
    display: flex;
    align-items: center;
    gap: 0.5rem;
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
    display: flex;
    align-items: center;
    justify-content: space-between; /* pushes arrow to the right */
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
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    grid-template-rows: repeat(3, 1fr);
    gap: 1.5rem;
    padding: 1.5rem;
    background: var(--bg-color);
    overflow-y: auto;
    height: calc(100vh - var(--navbar-height) - 2rem);
  }

  .msg-type-list::-webkit-scrollbar,
  .plots-grid::-webkit-scrollbar {
    width: 0; /* Remove vertical scrollbar */
    height: 0; /* Remove horizontal scrollbar if any */
  }

  @media (max-width: 1280px) {
    .plots-grid {
      gap: 1rem;
      padding: 1rem;
    }
  }

  @media (min-width: 1280px) {
    .msg-type-item {
      margin-bottom: 0.5rem;
    }
  }

  @media (min-width: 1920px) {
    .plots-grid {
      /* grid-template-columns: repeat(3, 1fr); */
    }
  }
</style>
