import { writable } from 'svelte/store';

export const enumMappings = writable({});

export async function loadEnums() {
    try {
        const response = await fetch('/enums');
        const enums = await response.json();
        enumMappings.set(enums);
    } catch (error) {
        console.error('Failed to load enums:', error);
        enumMappings.set({});
    }
}