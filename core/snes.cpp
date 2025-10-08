#define _GNU_SOURCE
#include <string.h>      // for strcasestr

#include "utils.h"
#include "cores.h"
#include "overlay.h"

bool srm_ok;
std::string srm_fname;
USB_NOCACHE_RAM_SECTION FIL f_srm;

int loadsnesbsram(const char* fname, unsigned int expected_filesize);

// return 0 if snes header is successfully parsed at off
// typ 0: LoROM, 1: HiROM, 2: ExHiROM
int parse_snes_header(FIL *fp, int pos, int file_size, int typ, unsigned char *hdr,
                      int *map_ctrl, int *rom_type_header, int *rom_size,
                      int *ram_size, int *company) {
    unsigned int br;
    if (f_lseek(fp, pos))
        return 1;
    f_read(fp, hdr, 64, &br);
    if (br != 64) return 1;
    int mc = hdr[21];
    int rom = hdr[23];
    int ram = hdr[24];
    int checksum = (hdr[28] << 8) + hdr[29];
    int checksum_compliment = (hdr[30] << 8) + hdr[31];
    int reset = (hdr[61] << 8) + hdr[60];
    int size2 = 1024 << rom;

    overlay_status("size=%d", size2);

    // calc heuristics score
    int score = 0;		
    if (size2 >= file_size) score++;
    if (rom == 1) score++;
    if (checksum + checksum_compliment == 0xffff) score++;
    int all_ascii = 1;
    for (int i = 0; i < 21; i++)
        if (hdr[i] < 32 || hdr[i] > 127)
            all_ascii = 0;
    score += all_ascii;

    overlay_status("pos=%x, type=%d, map_ctrl=%d, rom=%d, ram=%d, checksum=%x, checksum_comp=%x, reset=%x, score=%d\n", 
            pos, typ, mc, rom, ram, checksum, checksum_compliment, reset, score);

    if (rom < 14 && ram <= 7 && score >= 1 && 
        reset >= 0x8000 &&				// reset vector position correct
       ((typ == 0 && (mc & 3) == 0) || 	// normal LoROM
        (typ == 0 && mc == 0x53)    ||	// contra 3 has 0x53 and LoROM
        (typ == 1 && (mc & 3) == 1) ||	// HiROM
        (typ == 2 && (mc & 3) == 2))) {	// ExHiROM
        *map_ctrl = mc;
        *rom_type_header = hdr[22];
        *rom_size = rom;
        *ram_size = ram;
        *company = hdr[26];
        return 0;
    }
    return 1;
}

// TODO: implement bsram backup
// return 0 if successful
int loadsnes(const char *fname) {
    int r = 1;
    DEBUG("loadsnes start");

    // check extension .sfc or .smc
    char *p = strcasestr(fname, ".sfc");
    if (p == NULL)
        p = strcasestr(fname, ".smc");
    if (p == NULL) {
        overlay_message("Only .smc or .sfc supported", 1);
        return r;
    }

    r = f_open(&fcore, fname, FA_READ);
    if (r) {
        overlay_status("Cannot open file");
        return r;
    }
    unsigned int br, total = 0;
    int size = get_file_size(fname);
    int map_ctrl, rom_type_header, rom_size, ram_size, company;
    // parse SNES header from ROM file
    int off = size & 0x3ff;		// rom header (0 or 512)
    int header_pos;
    overlay_status("snes rom header offset: %d\n", off);
    
    header_pos = 0x7fc0 + off;
    if (parse_snes_header(&fcore, header_pos, size-off, 0, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
        header_pos = 0xffc0 + off;
        if (parse_snes_header(&fcore, header_pos, size-off, 1, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
            header_pos = 0x40ffc0 + off;
            if (parse_snes_header(&fcore, header_pos, size-off, 2, fbuf, &map_ctrl, &rom_type_header, &rom_size, &ram_size, &company)) {
                overlay_status("Not a SNES ROM file");
                delay(200);
                goto loadsnes_close_file;
            }
        }
    }

    // load actual ROM
    set_loading_state(1);		// enable game loading, this resets SNES
    core_running = false;

    // Send 64-byte header to snes
    send_fbuf_data(64);

    // Send rom content to snes
    if ((r = f_lseek(&fcore, off)) != FR_OK) {
        overlay_status("Seek failure");
        goto loadsnes_snes_end;
    }
    do {
        if ((r = f_read(&fcore, fbuf, 1024, &br)) != FR_OK)
            break;
        if (br == 0) break;
        send_fbuf_data(br);
        total += br;
        if ((total & 0xffff) == 0) {	// display progress every 64KB
            overlay_status("%d/%dK", total >> 10, size >> 10);
            if ((map_ctrl & 3) == 0)
                overlay_printf(" Lo");
            else if ((map_ctrl & 3) == 1)
                overlay_printf(" Hi");
            else if ((map_ctrl & 3) == 2)
                overlay_printf(" ExHi");
            //              01234567890123456789012345678901
            overlay_printf(" ROM=%d RAM=%d                 ", 1 << rom_size, ram_size ? (1 << ram_size) : 0);
        }
    } while (br == 1024);

    overlay_status("Success");

    // load SRM from memory
    if(ram_size != 0) {
        std::string rom_name{fname};
        std::string save_dir = rom_name.substr(0, rom_name.find_last_of('/'));
        save_dir.append("/saves/");
        int rdir = f_mkdir(save_dir.c_str());
        if(rdir != FR_OK && rdir != FR_EXIST) {
            // saves directory could not be created
            overlay_message("Could not create saves directory",1);
        }

        std::string rom_title{rom_name, rom_name.find_last_of('/')+1};
        rom_title.erase(rom_title.find_last_of('.'));

        srm_fname.clear();
        srm_fname.append(save_dir);
        srm_fname.append(rom_title);
        srm_fname.append(".srm");
        
        loadsnesbsram(srm_fname.c_str(), ((1 << ram_size)*1024));
    }

    core_running = true;

    overlay(0);		// turn off OSD

loadsnes_snes_end:
    set_loading_state(0);	// turn off game loading, this starts SNES
loadsnes_close_file:
    f_close(&fcore);
    return r;
}

const int BSRAM_CHUNK_SIZE = 512;

int loadsnesbsram(const char* fname, unsigned int expected_filesize) {
    int r = 1;
    srm_ok = false;

    DEBUG("loadsnes bsram begin");

    // check extension is .srm (bsram save file)
    char *p = strcasestr(fname, ".srm");
    if(p == NULL) {
        overlay_message("Only .srm files supported", 1);
        return r;
    }
    FILINFO fno;
    r = f_stat(fname, &fno);
    if(FR_NO_FILE == r) {
        overlay_status("Creating new %dK srm file", expected_filesize >> 10);
        r = f_open(&f_srm, fname, FA_WRITE|FA_CREATE_ALWAYS);
        if(FR_OK != r) {
            overlay_status("Failed creating .srm file");
            return r;
        }
        unsigned int bytes_written = 0;
        memset(fbuf, 0, BSRAM_CHUNK_SIZE);
        for(unsigned int i = 0; i < expected_filesize; i += BSRAM_CHUNK_SIZE) {
            r = f_write(&f_srm, fbuf, BSRAM_CHUNK_SIZE, &bytes_written);
            if((FR_OK != r) || (BSRAM_CHUNK_SIZE != bytes_written)) {
                overlay_status(".srm file write failure");
                return r;
            }
        }
        r = f_close(&f_srm);
        srm_ok = true;
        DEBUG("srm file okay");
        if(FR_OK != r) {
            overlay_status (".srm file close failure");
            return r;
        }
    } 
    else if(FR_OK == r) {
        auto filelen = fno.fsize;
        if(filelen != expected_filesize) {
            // Just a warning, don't stop loading the file
            //              01234567890123456789012345678901
            overlay_status("SRM Length %dK != expect %dK", filelen>>10, expected_filesize>>10);
        }
        r = f_open(&f_srm, fname, FA_READ);
        if(FR_OK != r) {
            overlay_status("Cannot open SRM file");
            return r;
        }
        srm_ok = true;
        DEBUG("srm file okay");

        unsigned int bytes_read = 0;
        unsigned int total = 0;
        do {
            if((r = f_read(&f_srm, fbuf, BSRAM_CHUNK_SIZE, &bytes_read)) != FR_OK){
                break;
            }
            if(bytes_read == 0) {
                break;
            }

            // Send bsram address and data for this chunk
            // block num = total / 512
            unsigned int block_num = total >> 9;
            taskENTER_CRITICAL();
            fpga_tx_header(0x0E, BSRAM_CHUNK_SIZE+3); // 0x0e blocknum[15:0] <data> send a sector (512 bytes) of data to BSRAM memory
                                                      // first 2 bytes = block number, followed by 512 bytes of data
            fpga_tx_byte(block_num >> 8);
            fpga_tx_byte(block_num & 0xFF);
            for (int i = 0; i < BSRAM_CHUNK_SIZE; i ++) {
                fpga_tx_byte(fbuf[i]);
            }
            taskEXIT_CRITICAL();
            total += bytes_read;
            if((total & 0x1FFF) == 0) { // display every 8KB of progress
                overlay_status("RAM: %d/%dK", total>>10, (int)(filelen >> 10));
            }
        } while(bytes_read == BSRAM_CHUNK_SIZE);

        f_close(&f_srm);
    }    

    return r;
}

// SnesMenu implementation
SnesMenu::SnesMenu(const char *imgdir) : imgdir(imgdir) {}

void SnesMenu::render() {
    overlay_clear();
    overlay_cursor(0, 10);
    //              012345678901234567890123456789012
    overlay_printf("          --- SNES ---          \n");
    overlay_cursor(0, 13);
    overlay_printf("  << Main Menu\n");
}

std::vector<int> SnesMenu::get_options() {
    return {13};
}

bool SnesMenu::on_choose(int idx) {

    return true;
}
