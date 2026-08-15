<script>
    // State variables for the calibration form
    let angle = 0.0;
    let statusMessage = "";
    let isError = false;
    let isLoading = false;
  
    async function sendAngle() {
      // Basic validation
      if (angle === null || angle === undefined) {
        statusMessage = "Please enter a valid angle.";
        isError = true;
        return;
      }
  
      isLoading = true;
      statusMessage = "Sending...";
      isError = false;
  
      try {
        const response = await fetch(`http://${import.meta.env.VITE_BACKEND_URL}/calibrate/lean_angle`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ angle: parseFloat(angle) }),
        });
  
        if (!response.ok) {
          throw new Error(`Server error: ${response.status}`);
        }
  
        const data = await response.json();
        statusMessage = data.message || "Angle sent successfully!";
        isError = false;
        
      } catch (error) {
        console.error("Error sending angle:", error);
        statusMessage = "Failed to send angle. Is the backend running?";
        isError = true;
      } finally {
        isLoading = false;
      }
    }
  </script>
  
  <div class="main-container">
    <div class="calibration-card">
      <h2>Lean Angle Calibration</h2>
      
      <div class="input-group">
        <label for="angle">Set Angle:</label>
        <input 
          id="angle" 
          type="number" 
          step="0.1" 
          bind:value={angle} 
          disabled={isLoading}
          placeholder="Enter angle (e.g., 12.5)"
        />
      </div>
  
      <button on:click={sendAngle} disabled={isLoading}>
        {isLoading ? "Sending..." : "Send Angle"}
      </button>
  
      {#if statusMessage}
        <div class="status-message" class:error={isError} class:success={!isError && !isLoading}>
          {statusMessage}
        </div>
      {/if}
    </div>
  </div>
  
  <style>
    .main-container {
      display: flex;
      justify-content: center;
      align-items: center;
      margin: 0 auto;
      margin-top: var(--navbar-height, 60px);
      height: calc(100vh - var(--navbar-height, 60px));
      padding: 2rem;
      box-sizing: border-box;
      overflow: hidden;
      background-color: var(--bg-color);
    }
  
    .calibration-card {
      display: flex;
      flex-direction: column;
      gap: 20px;
      width: 100%;
      max-width: 400px;
      padding: 30px;
      background-color: var(--snd-bg-color);
      border-radius: 12px;
      box-shadow: 0 8px 16px rgba(0, 0, 0, 0.1);
    }
  
    .calibration-card h2 {
      margin: 0;
      font-size: 1.5rem;
      color: var(--text-color);
      text-align: center;
    }
  
    .input-group {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
  
    .input-group label {
      font-size: 1rem;
      color: var(--text-color);
      font-weight: 600;
    }
  
    .input-group input {
      padding: 12px;
      border: 1px solid #ccc;
      border-radius: 6px;
      font-size: 1.1rem;
      transition: border-color 0.2s;
    }
  
    .input-group input:focus {
      outline: none;
      border-color: #ff965f;
      box-shadow: 0 0 0 3px rgba(0, 123, 255, 0.1);
    }
  
    button {
      padding: 12px;
      background-color: #ff965f;
      color: #111217;
      border: none;
      border-radius: 6px;
      font-size: 1.1rem;
      font-weight: 600;
      cursor: pointer;
      transition: background-color 0.2s, transform 0.1s;
    }
  
    button:hover:not(:disabled) {
      background-color: #ff965f;
      opacity: 0.75;
    }
  
    button:active:not(:disabled) {
      transform: translateY(1px);
    }
  
    button:disabled {
      background-color: #a0c4ff;
      cursor: not-allowed;
    }
  
    .status-message {
      padding: 12px;
      border-radius: 6px;
      font-size: 1rem;
      text-align: center;
      font-weight: 500;
    }
  
    .success {
      background-color: #d4edda;
      color: #388729;
      border: 1px solid #c3e6cb;
    }
  
    .error {
      background-color: #f8d7da;
      color: #c41934;
      border: 1px solid #f5c6cb;
    }
  </style>