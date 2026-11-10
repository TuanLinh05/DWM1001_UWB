"""Rebuild a .ulg file from PX4 MAVLink log streaming (LOGGING_DATA[_ACKED]).

Follows PX4 Tools/mavlink_ulog_streaming.py: the first 16 bytes are the ULog
file header; afterwards every chunk carries `first_message_offset`, the start
of the first ULog message inside the chunk (255 = none starts here). After a
sequence gap, the partial message is discarded and writing resumes at the next
message boundary; a ULog dropout message ('O') records the gap in the data
section. The header/definition section is sent ACKED (reliable), so it is
complete; only the data section can have dropouts.

ULog message: uint16 size (payload only), uint8 type, payload.
"""
from __future__ import annotations

from typing import BinaryIO

ULOG_HEADER_LEN = 16
ULOG_MSG_HEADER_LEN = 3
NO_MESSAGE_START = 255


class UlogStreamWriter:
    def __init__(self, fh: BinaryIO) -> None:
        self.fh = fh
        self.last_sequence = -1
        self.got_ulog_header = False
        self.got_header_section = False
        self.pending = bytearray()
        self.chunks = 0
        self.duplicates = 0
        self.dropped_chunks = 0
        self.dropout_messages = 0
        self.bytes_written = 0

    def _check_sequence(self, seq: int) -> tuple[bool, int]:
        """(is_newer, number of chunks lost before it); 16-bit wrap-around."""
        if self.last_sequence == -1:
            return True, 0
        if seq == self.last_sequence:
            return False, 0
        if seq > self.last_sequence:
            if seq - self.last_sequence > (1 << 15):
                return False, 0  # old packet from before a wrap
            return True, seq - self.last_sequence - 1
        if self.last_sequence - seq > (1 << 15):
            return True, (1 << 16) - self.last_sequence - 1 + seq
        return False, 0

    def _write(self, data: bytes | bytearray) -> None:
        if data:
            self.fh.write(bytes(data))
            self.bytes_written += len(data)

    def _write_messages(self, data: bytes | bytearray) -> bytearray:
        """Write complete ULog messages from the start of `data`; return the rest."""
        view = bytearray(data)
        pos = 0
        while len(view) - pos > 2:
            length = view[pos] + view[pos + 1] * 256 + ULOG_MSG_HEADER_LEN
            if length > len(view) - pos:
                break
            self._write(view[pos:pos + length])
            pos += length
        return view[pos:]

    def handle(self, sequence: int, data: bytes, first_message_offset: int, acked: bool) -> bool:
        """Process one LOGGING_DATA payload (already cut to its `length`).

        Returns False for duplicates/old chunks (the caller must still ACK an
        ACKED chunk, so PX4 stops retransmitting it).
        """
        is_newer, drops = self._check_sequence(sequence)
        if not is_newer:
            self.duplicates += 1
            return False
        self.last_sequence = sequence
        self.chunks += 1
        if not acked:
            self.got_header_section = True  # data section is streamed un-acked
        data = bytes(data)

        if not self.got_ulog_header:
            if len(data) < ULOG_HEADER_LEN:
                raise ValueError("first ULog chunk shorter than the file header")
            self._write(data[:ULOG_HEADER_LEN])
            data = data[ULOG_HEADER_LEN:]
            if first_message_offset != NO_MESSAGE_START:
                first_message_offset = max(0, first_message_offset - ULOG_HEADER_LEN)
            self.got_ulog_header = True

        if drops > 0:
            self.dropped_chunks += drops
            if self.got_header_section:
                # Duration unknown: PX4 tooling assumes 10 ms per lost chunk.
                duration_ms = min(drops, 25) * 10
                self._write(bytes((2, 0, ord("O"), duration_ms & 0xFF, duration_ms >> 8)))
                self.dropout_messages += 1
            self.pending = bytearray()  # the message across the gap is incomplete
            if first_message_offset == NO_MESSAGE_START:
                return True
            data = data[first_message_offset:]
            first_message_offset = 0

        if first_message_offset == NO_MESSAGE_START:
            # Inside one long message. With nothing pending we lost its start
            # (after a gap): discard until the next message boundary.
            if self.pending:
                self.pending.extend(data)
                self.pending = self._write_messages(self.pending)  # may complete it
            return True
        if self.pending:
            self._write(self.pending + data[:first_message_offset])
            self.pending = bytearray()
        self.pending = self._write_messages(data[first_message_offset:])
        return True
