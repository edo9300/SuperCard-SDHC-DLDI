/*
	iointerface.c template
	
 Copyright (c) 2006 Michael "Chishm" Chisholm
	
 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

  1. All derivative works must be clearly marked as such. Derivatives of this file 
	 must have the author of the derivative indicated within the source.  
  2. The name of the author may not be used to endorse or promote products derived
     from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdbool.h>
#include <stdint.h>

#include "scsd/new_scsdio.h"
#include "scsd/sc_commands.h"
#include "scsd/sd.h"

const char build_info[]=
"SuperCard-SDHC-DLDI Ver 1.0 The 'Moon Eclipse' "
"[https://github.com/ArcheyChen/SuperCard-SDHC-DLDI]"
"Author: Ausar Date:2024/03/03, edo9300 Date:2024/10/25\n"
"Supports:1.SD init 2.SDHC 3.unaligned r/w 4.compatiable with nds-bootstrap 5.compatible with Supercard Lite"
"Please Keep this info";

const char* get_build_info(){
    return build_info;
}

bool startup(void) {
    return MemoryCard_IsInserted() && SDInit();
}

bool isInserted(void) {
    return MemoryCard_IsInserted();
}

bool clearStatus(void) {
    return true;
}

bool readSectors(uint32_t sector, uint32_t numSectors, void* buff) {
	return ReadSector(buff, sector, numSectors);
}

bool writeSectors(uint32_t sector, uint32_t numSectors, void* buffer) {
    WriteSector(buffer, sector, numSectors);
    return true;
}

bool shutdown(void) {
    return true;
}


