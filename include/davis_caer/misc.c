/*
 * misc.c
 *
 *  Created on: Dec 9, 2013
 *      Author: chtekk
 */

#include "misc.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

void caerDaemonize(void) {
	// Double fork to background, for more details take a look at:
	// http://stackoverflow.com/questions/3095566/linux-daemonize
	pid_t result = fork();

	// Handle errors first.
	if (result == -1) {
		caerLog(LOG_EMERGENCY, "Daemonize", "Failed the first fork.");
		exit(EXIT_FAILURE);
	}

	// Then exit if you are the parent.
	if (result > 0) {
		// Exit without calling atexit() functions!
		_exit(EXIT_SUCCESS);
	}

	// So we must be the child here (result == 0).
	// Become session group leader.
	setsid();

	// Fork again, so that the child can never regain a controlling terminal.
	result = fork();

	// Handle errors first.
	if (result == -1) {
		caerLog(LOG_EMERGENCY, "Daemonize", "Failed the second fork.");
		exit(EXIT_FAILURE);
	}

	// Then exit if you are the parent.
	if (result > 0) {
		// Exit without calling atexit() functions!
		_exit(EXIT_SUCCESS);
	}

	// So we must be the child here (result == 0).
	// Ensure we don't keep directories busy.
	if (chdir("/") != 0) {
		caerLog(LOG_EMERGENCY, "Daemonize", "Failed to change directory to '/'.");
		exit(EXIT_FAILURE);
	}

	// Close stdin, stdout and stderr fds, disable console logging in caerLog().
	caerLogDisableConsole();

	close(0); // stdin
	close(1); // stdout
	close(2); // stderr

	// Redirect stdin to /dev/null and stdout/stderr to the log-file, just to be sure.
	if (open("/dev/null", O_RDONLY) != 0) {
		caerLog(LOG_EMERGENCY, "Daemonize", "Failed to redirect stdin to log file.");
		exit(EXIT_FAILURE);
	}
	if (dup2(3, 1) != 1) {
		caerLog(LOG_EMERGENCY, "Daemonize", "Failed to redirect stdout to log file.");
		exit(EXIT_FAILURE);
	}
	if (dup2(3, 2) != 2) {
		caerLog(LOG_EMERGENCY, "Daemonize", "Failed to redirect stderr to log file.");
		exit(EXIT_FAILURE);
	}

	// At this point everything should be ok and we can return!
}

void caerBitArrayCopy(uint8_t *src, size_t srcPos, uint8_t *dest, size_t destPos, size_t length) {
	size_t copyOffset = 0;

	while (copyOffset < length) {
		size_t srcBytePos = (srcPos + copyOffset) >> 3;
		size_t srcBitPos = (srcPos + copyOffset) & 0x07;
		uint8_t srcBitMask = U8T(0x80 >> srcBitPos);

		size_t destBytePos = (destPos + copyOffset) >> 3;
		size_t destBitPos = (destPos + copyOffset) & 0x07;
		uint8_t destBitMask = U8T(0x80 >> destBitPos);

		if ((src[srcBytePos] & srcBitMask) != 0) {
			// Set bit.
			dest[destBytePos] |= destBitMask;
		}
		else {
			// Clear bit.
			dest[destBytePos] &= U8T(~destBitMask);
		}

		copyOffset++;
	}
}
