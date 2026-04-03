export function useLocalStorage(key, defaultValue) {
    const stored = localStorage.getItem(key);
    const initial = stored ? JSON.parse(stored) : defaultValue;

    return {
        get: () => localStorage.getItem(key),
        set: (value) => localStorage.setItem(key, JSON.stringify(value)),
        remove: () => localStorage.removeItem(key)
    };
}