/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#ifndef SHENG_GMU_COMMAND_DB_H
#define SHENG_GMU_COMMAND_DB_H
#include "ShengGmuVotes.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {ShengGmuDbSuccess,ShengGmuDbArgument,ShengGmuDbVersion,
    ShengGmuDbCorrupt,ShengGmuDbDuplicate,ShengGmuDbMissing} ShengGmuDbStatus;
/* Parse an immutable CPU-readable snapshot of the complete pinned DT command
 * DB reservation. No mapping, MMIO or allocation. Caller must obtain/retain
 * actual platform-owned bytes coherently and bind expected_version from an
 * independently qualified platform policy; reading that word and passing it
 * back is not version qualification. No version is guessed here.
 * The format's checksum is unused by the source; this parser does NOT verify
 * provenance/integrity/authenticity. All offsets/counts are checked, with a
 * 4096-entry bound and unique target records. Missing GMxC is authoritative
 * only after the complete structurally valid database has been scanned.
 * Output contains aliases into input, must be disjoint and retained/immutable
 * through startup configuration construction. Failure leaves output unchanged.
 * Output must be initialized. Chip identity and optional features are caller
 * policy; this sets
 * only cmd-db fields and preserves those policy fields. No hardware readiness
 * follows from parsing, and malformed data never permits a power operation. */
ShengGmuDbStatus ShengGmuCommandDbInputs(const uint8_t *,size_t,uint32_t expected_version,ShengGmuVoteInputs *);
#ifdef __cplusplus
}
#endif
#endif
