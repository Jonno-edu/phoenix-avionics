// worker.js
let cursors = {};
let pollRate = 250;
let isConnected = false;

self.onmessage = function(e) {
    if (e.data.type === 'INIT') {
        cursors = e.data.cursors;
    } else if (e.data.type === 'SET_RATE') {
        pollRate = e.data.rate;
    } else if (e.data.type === 'STATUS') {
        isConnected = e.data.connected;
    }
};

async function fetchData() {
    if (!isConnected) {
        setTimeout(fetchData, 1000);
        return;
    }

    try {
        let query = Object.keys(cursors)
            .map(k => `since_${k}=${encodeURIComponent(cursors[k])}`)
            .join('&');
            
        const response = await fetch('/api/history?' + query);
        const history = await response.json();

        let hasNewData = false;
        Object.keys(history).forEach(id => {
            if (history[id].length > 0) {
                hasNewData = true;
                cursors[id] = history[id][history[id].length - 1].Time;
            }
        });

        if (hasNewData) {
            self.postMessage({ type: 'DATA_UPDATE', history: history });
        }
    } catch (e) {
        console.error("Worker fetch error:", e);
    }

    setTimeout(fetchData, pollRate);
}

// Start the loop
fetchData();
