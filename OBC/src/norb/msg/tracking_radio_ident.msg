# tracking_radio_ident.msg
# Tracking radio node identification response. Published by housekeeping module.
# Shares the same ICD type as eps_ident — node_type field distinguishes them.

@include <phoenix_icd.h>
@extern TlmIdentificationPayload_t
