document.addEventListener("DOMContentLoaded", async () => {
    const sessionsList = document.getElementById("sessions-list");

    try {
        // Fetch sessions for the currently logged-in user
        const response = await fetch("/sessions");  // endpoint we will create
        const data = await response.json();

        if (!response.ok) {
            sessionsList.innerHTML = `<li>Error: ${data.detail}</li>`;
            return;
        }

        if (data.sessions.length === 0) {
            sessionsList.innerHTML = "<li>No sessions found</li>";
            return;
        }

        // Populate the list
        sessionsList.innerHTML = data.sessions
            .map(
                (s) =>
                    `<li>
                        Session ID: ${s.id} | Start: ${new Date(
                        s.start_time
                    ).toLocaleString()} | End: ${
                        s.end_time
                            ? new Date(s.end_time).toLocaleString()
                            : "Ongoing"
                    }
                    </li>`
            )
            .join("");

    } catch (err) {
        console.error(err);
        sessionsList.innerHTML = `<li>Network error: ${err.message}</li>`;
    }
});
