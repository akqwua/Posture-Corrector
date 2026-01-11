// app.js
// sanity check
// Wait until the DOM is fully loaded

document.addEventListener("DOMContentLoaded", () => {
    const loginForm = document.getElementById("login-form");
    const loginMsg = document.getElementById("login-msg");

    if (!loginForm) return;

    loginForm.addEventListener("submit", async (e) => {
        e.preventDefault();

        const username = document.getElementById("username").value;
        const password = document.getElementById("password").value;

        try {
            const response = await fetch("/login", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ username, password }),
            });

            const data = await response.json();

            if (response.ok) {
                // Login successful → redirect to dashboard
                console.log("Login successful!", data);
                loginMsg.innerText = `Welcome, ${data.username}!`;

                // Redirect
                window.location.href = "/frontend/dashboard.html";
            } else {
                console.error("Login failed:", data.detail);
                loginMsg.innerText = `Login failed: ${data.detail}`;
            }
        } catch (err) {
            console.error("Network error:", err);
            loginMsg.innerText = `Network error: ${err.message}`;
        }
    });
});
