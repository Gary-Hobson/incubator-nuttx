/****************************************************************************
 * drivers/note/noteram_driver.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/segger/rtt.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  TRACE_TYPE_LTTNG_KERNEL = 0,  /* Common Trace Format : Linux Kernel Trace */
  TRACE_TYPE_GENERIC_CTF  = 1,  /* Common Trace Format : Generic CTF Trace */
  TRACE_TYPE_LTTNG_UST    = 2,  /* Common Trace Format : LTTng UST Trace */
  TRACE_TYPE_CUSTOM_TEXT  = 3,  /* Custom Text :         TmfGeneric */
  TRACE_TYPE_CUSTOM_XML   = 4,  /* Custom XML :          Custom XML Log */
  TRACE_TYPE_ANDROID      = 5,  /* Custom Format :       Android ATrace */
} trace_dump_t;

int trace_dump(trace_dump_t type, FAR FILE *out);;

static int rtt_note_init(FAR struct note_channels_s *channel)
{
  FAR struct lib_rttoutstream_s *stream =
      kmm_malloc(sizeof(struct lib_rttoutstream_s));
  // lib_rttoutstream_open(stream, 1, 1024);
  FILE * fp = fopen("/dev/rtt1", "w+");
  channel->priv = fp;
  return 0;
}

static void rtt_note_add(FAR struct note_channels_s *channel,
                         FAR const void *note, size_t len)
{
  FILE * fp = channel->priv;
  fwrite(note, 1, len, fp);
  // trace_dump( , FAR FILE *out)
  // struct trace_dump_context_s ctx;
  uint8_t tracedata[UCHAR_MAX];
  FAR uint8_t *p;
  int size;
  int ret;
  int fd;
 trace_dump_once(TRACE_TYPE_ANDROID, fp, note);

  // trace_dump_init_context(&ctx, fd);


  // trace_dump_one(TRACE_TYPE_ANDROID, fp, p, &ctx);
}

struct note_channels_s g_rtt_channels = {
  .write = rtt_note_add,
  .init = rtt_note_init,
  .filter_flag = NOTE_FILTER(NOTE_START) | NOTE_FILTER(NOTE_STOP),
};
