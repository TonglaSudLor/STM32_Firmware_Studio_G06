let inputBuffer = "";

onmessage = async function(e) {
    if (e.data.type === 'start') {
        const reader = e.data.readable.getReader();
        const decoder = new TextDecoder();

        try {
            while (true) {
                const { value, done } = await reader.read();
                if (done) break;

                // value is a Uint8Array if we don't use TextDecoderStream in the main thread
                // But if we pipe it in the main thread, we send the readable.
                // It's better to pipe in the worker.
                inputBuffer += decoder.decode(value, { stream: true });

                if (inputBuffer.length > 10000) {
                    const lastDollar = inputBuffer.lastIndexOf('$');
                    inputBuffer = lastDollar !== -1 ? inputBuffer.substring(lastDollar) : "";
                }

                let s = inputBuffer.indexOf('$');
                let eIdx = inputBuffer.indexOf('*');
                while (s !== -1 && eIdx !== -1 && eIdx > s) {
                    const packet = inputBuffer.substring(s + 1, eIdx);
                    postMessage({ type: 'packet', data: packet });
                    inputBuffer = inputBuffer.substring(eIdx + 1);
                    s = inputBuffer.indexOf('$');
                    eIdx = inputBuffer.indexOf('*');
                }

                const dollarPos = inputBuffer.indexOf('$');
                const plainPart = dollarPos !== -1 ? inputBuffer.substring(0, dollarPos) : inputBuffer;
                const lastNewline = plainPart.lastIndexOf('\n');
                if (lastNewline !== -1) {
                    plainPart.substring(0, lastNewline).split('\n').forEach(line => {
                        const clean = line.replace(/\r/g, '').trim();
                        if (clean.startsWith('[')) {
                            postMessage({ type: 'log', data: clean });
                        }
                    });
                    inputBuffer = inputBuffer.substring(lastNewline + 1);
                }
            }
        } catch (err) {
            postMessage({ type: 'error', data: err.message });
        }
    }
};
