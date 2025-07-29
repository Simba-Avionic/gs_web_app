import { writable } from 'svelte/store';

const stored = localStorage.getItem('theme');
const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
const initial = stored || (prefersDark ? 'dark' : 'light');

export const theme = writable(initial);

theme.subscribe((value) => {
  document.documentElement.setAttribute('data-theme', value);
  localStorage.setItem('theme', value);
});
