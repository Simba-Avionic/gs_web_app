const API_BASE = `http://${window.location.host}`;
let cachedTopics = null;

export async function fetchConfig(forceRefresh = true) {
    if (!forceRefresh && cachedTopics) {
        return cachedTopics;
    }

    try {
        const savedTopics = localStorage.getItem("topics");
        if (savedTopics && !forceRefresh) {
            try {
                const parsedTopics = JSON.parse(savedTopics);
                if (parsedTopics && Array.isArray(parsedTopics) && parsedTopics.length > 0) {
                    cachedTopics = parsedTopics;
                    console.log("Loaded topics from LocalStorage");
                    return parsedTopics;
                }
            } catch (parseError) {
                console.error("Error parsing saved topics:", parseError);
            }
        }
    } catch (storageError) {
        console.error("LocalStorage error:", storageError);
    }

    try {
        const response = await fetch(`${API_BASE}/config`);
        const data = await response.json();
        cachedTopics = data.topics;
        return data.topics;
    } catch (error) {
        console.error("Error fetching config:", error);
        return [];
    }
}

export async function fetchMessageDefs() {
    try {
        const response = await fetch(`${API_BASE}/config`);
        const data = await response.json();
        return data.msg_defs || [];
    } catch (error) {
        console.error("Error fetching message definitions:", error);
        return [];
    }
}

export async function queryFieldData(topic, fieldName, timeRange) {
    const params = new URLSearchParams({
        field_name: fieldName,
        time_range: timeRange
    });
    return fetch(`${API_BASE}/${topic}/query?${params}`).then(r => r.json());
}