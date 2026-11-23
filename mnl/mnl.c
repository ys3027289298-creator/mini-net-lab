/*
 * mnl.c - unified failure sink implementation shared by all four modules.
 */
#include "mnl.h"

#include <stdio.h>

void mnl_sink_init(mnl_sink_t* sk) {
    if (sk == NULL) return;
    sk->count = 0;
    sk->cur_index = -1;
}

void mnl_sink_begin(mnl_sink_t* sk, int entry_index) {
    if (sk != NULL) sk->cur_index = entry_index;
}

int mnl_report(mnl_sink_t* sk, mnl_module_t mod, mnl_field_t field,
               mnl_reason_t reason) {
    if (sk != NULL && sk->count < MNL_ERR_CAP) {
        mnl_err_t* e = &sk->errs[sk->count++];
        e->module = mod;
        e->field = field;
        e->reason = reason;
        e->entry_index = sk->cur_index;
    }
    return -1;
}

const char* mnl_module_name(mnl_module_t m) {
    switch (m) {
    case MNL_MOD_SNAPSHOT:  return "snapshot";
    case MNL_MOD_CLOCK:     return "clock";
    case MNL_MOD_AOI:       return "aoi";
    case MNL_MOD_RECONCILE: return "reconcile";
    default:                return "unknown";
    }
}

const char* mnl_field_name(mnl_field_t f) {
    switch (f) {
    case MNL_F_NONE:         return "none";
    case MNL_F_SNAP_STATE:   return "state";
    case MNL_F_SNAP_PACKET:  return "packet";
    case MNL_F_SNAP_OUT:     return "out";
    case MNL_F_POS_SCALE:    return "pos_scale";
    case MNL_F_TICK:         return "tick";
    case MNL_F_X:            return "x";
    case MNL_F_Y:            return "y";
    case MNL_F_PACKET_BYTES: return "packet_bytes";
    case MNL_F_SYNC:         return "sync";
    case MNL_F_SAMPLE:       return "sample";
    case MNL_F_T_SEND:       return "t_send";
    case MNL_F_T_RECV:       return "t_recv";
    case MNL_F_REMOTE_TIME:  return "remote_time";
    case MNL_F_WORLD:        return "world";
    case MNL_F_CONFIG:       return "config";
    case MNL_F_CELL:         return "cell";
    case MNL_F_RADIUS:       return "radius";
    case MNL_F_ID:           return "id";
    case MNL_F_POINT:        return "point";
    case MNL_F_QUERY:        return "query";
    case MNL_F_CAP:          return "cap";
    case MNL_F_PREV:         return "prev";
    case MNL_F_CUR:          return "cur";
    case MNL_F_DIFF:         return "diff";
    case MNL_F_RECON:        return "recon";
    case MNL_F_RECON_POINT:  return "point";
    default:                 return "unknown";
    }
}

const char* mnl_reason_name(mnl_reason_t r) {
    switch (r) {
    case MNL_R_NONE:      return "none";
    case MNL_R_NULL:      return "null";
    case MNL_R_RANGE:     return "range";
    case MNL_R_NONFINITE: return "nonfinite";
    case MNL_R_DUPLICATE: return "duplicate";
    case MNL_R_NOT_FOUND: return "not_found";
    case MNL_R_FULL:      return "full";
    case MNL_R_PARSE:     return "parse";
    case MNL_R_BAD_OP:    return "bad_op";
    default:              return "unknown";
    }
}

void mnl_format(const mnl_err_t* e, char* buf, size_t n) {
    if (buf == NULL || n == 0) return;
    snprintf(buf, n, "%s/%s/%s@entry%d",
             mnl_module_name(e->module), mnl_field_name(e->field),
             mnl_reason_name(e->reason), e->entry_index);
}
