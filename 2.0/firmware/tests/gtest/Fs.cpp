// Fs.cpp - Filesystem unit tests
//
// Copyright (C) 2018 Arribada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
extern "C" {
#include <stdint.h>
#include "unity.h"
#include "Mocksyshal_flash.h"
#include "fs.h"
#include "fs_priv.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <list>
#include <vector>

using std::list;
using std::vector;

#define FLASH_SIZE          (FS_PRIV_SECTOR_SIZE * FS_PRIV_MAX_SECTORS)
#define ASCII(x)            ((x) >= 32 && (x) <= 127) ? (x) : '.'

static bool trace_on;
static char flash_ram[FLASH_SIZE];

class FsTest : public ::testing::Test {

    virtual void SetUp() {
        Mocksyshal_flash_Init();

        trace_on = false;
        syshal_flash_read_StubWithCallback(syshal_flash_read_Callback);
        syshal_flash_write_StubWithCallback(syshal_flash_write_Callback);
        syshal_flash_erase_StubWithCallback(syshal_flash_erase_Callback);
        for (unsigned int i = 0; i < FLASH_SIZE; i++)
            flash_ram[i] = 0xFF;
    }

    virtual void TearDown() {
        Mocksyshal_flash_Verify();
        Mocksyshal_flash_Destroy();
    }

public:

    void SetSectorAllocCounter(uint8_t sector, uint32_t alloc_counter) {
        union {
            uint32_t *alloc_counter;
            char *buffer;
        } a;
        a.buffer = &flash_ram[(sector * FS_PRIV_SECTOR_SIZE) + FS_PRIV_ALLOC_COUNTER_OFFSET];
        *a.alloc_counter = alloc_counter;
    }

    void CheckSectorAllocCounter(uint8_t sector, uint32_t alloc_counter) {
        union {
            uint32_t *alloc_counter;
            char *buffer;
        } a;
        a.buffer = &flash_ram[(sector * FS_PRIV_SECTOR_SIZE) + FS_PRIV_ALLOC_COUNTER_OFFSET];
        EXPECT_EQ(alloc_counter, *a.alloc_counter);
    }

    void CheckFileId(uint8_t sector, uint8_t file_id)
    {
        EXPECT_EQ(file_id, flash_ram[(sector * FS_PRIV_SECTOR_SIZE)]);
    }

    void DumpFlash(uint32_t start, uint32_t sz) {
        for (unsigned int i = 0; i < sz/8; i++) {
            printf("%08x:", start + (8*i));
            for (unsigned int j = 0; j < 8; j++)
                printf(" %02x", (unsigned char)flash_ram[start + (8*i) + j]);
            printf("  ");
            for (unsigned int j = 0; j < 8; j++)
                printf("%c", ASCII((unsigned char)flash_ram[start + (8*i) + j]));
            printf("\n");
        }
    }

    static int syshal_flash_read_Callback(uint32_t device, void *dest, uint32_t address, uint32_t size, int cmock_num_calls)
    {
        //printf("syshal_flash_read(%08x,%u)\n", address, size);
        for (unsigned int i = 0; i < size; i++)
            ((char *)dest)[i] = flash_ram[address + i];

        return 0;
    }

    static int syshal_flash_write_Callback(uint32_t device, const void *src, uint32_t address, uint32_t size, int cmock_num_calls)
    {
        if (trace_on)
            printf("syshal_flash_write(%08x, %u)\n", address, size);
        for (unsigned int i = 0; i < size; i++)
        {
            /* Ensure no new bits are being set */
            if ((((char *)src)[i] & flash_ram[address + i]) ^ ((char *)src)[i])
            {
                printf("syshal_flash_write: Can't set bits from 0 to 1 (%08x: %02x => %02x)\n", address + i,
                        (uint8_t)flash_ram[address + i], (uint8_t)((char *)src)[i]);
                assert(0);
            }
            flash_ram[address + i] = ((char *)src)[i];
        }

        return 0;
    }

    static int syshal_flash_erase_Callback(uint32_t device, uint32_t address, uint32_t size, int cmock_num_calls)
    {
        /* Make sure address is sector aligned */
        if (address % FS_PRIV_SECTOR_SIZE || size % FS_PRIV_SECTOR_SIZE)
        {
            printf("syshal_flash_erase: Non-aligned address %08x", address);
            assert(0);
        }

        for (unsigned int i = 0; i < size; i++)
            flash_ram[address + i] = 0xFF;

        return 0;
    }
};

TEST_F(FsTest, ContainsPagesWithOneByteRemaining)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    uint8_t buf[512];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    // Each write has a 2 byte header, so writing PAGE_SIZE - 2 will leave
    // zero bytes surplus in the page
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, buf, FS_PRIV_PAGE_SIZE - 2, &wr));
    // Each write has a 2 byte header, so writing PAGE_SIZE - 3 will leave
    // one bytes surplus in the page
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, buf, FS_PRIV_PAGE_SIZE - 3, &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));

    // Now try to read the data back
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, FS_PRIV_PAGE_SIZE, &rd));
    EXPECT_EQ((uint32_t)FS_PRIV_PAGE_SIZE, rd);
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, FS_PRIV_PAGE_SIZE, &rd));
    EXPECT_EQ((uint32_t)FS_PRIV_PAGE_SIZE - 5, rd);
    EXPECT_EQ(FS_ERROR_END_OF_FILE, fs_read(handle, buf, FS_PRIV_PAGE_SIZE, &rd));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, FormatPreservesAllocationCounter)
{
    fs_t fs;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        CheckSectorAllocCounter(i, 0);
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        CheckSectorAllocCounter(i, 1);
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        CheckSectorAllocCounter(i, 2);
}

TEST_F(FsTest, CannotUseBadDeviceIdentifier)
{
    fs_t fs;

    EXPECT_EQ(FS_ERROR_BAD_DEVICE, fs_init(FS_PRIV_MAX_DEVICES));
    EXPECT_EQ(FS_ERROR_BAD_DEVICE, fs_mount(FS_PRIV_MAX_DEVICES, &fs));
}

TEST_F(FsTest, SimpleFileIO)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
    };
    char buf[256];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), rd);
    EXPECT_EQ(0, strncmp(test_string[0], buf, strlen(test_string[0])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, CannotReadPastEndOfFile)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
    };
    char buf[256];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), rd);
    EXPECT_EQ(0, strncmp(test_string[0], buf, strlen(test_string[0])));
    EXPECT_EQ(FS_ERROR_END_OF_FILE, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, FileUserFlagsArePreserved)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7, rd_user_flags;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, &rd_user_flags));
    EXPECT_EQ(wr_user_flags, rd_user_flags);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, StatExistingFileAttributesArePreserved)
{
    fs_t fs;
    fs_handle_t handle;
    fs_stat_t stat;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(fs, 0, &stat));
    EXPECT_EQ(wr_user_flags, stat.user_flags);
    EXPECT_FALSE(stat.is_circular);
    EXPECT_FALSE(stat.is_protected);

    // Size is the actual size on disk and not the number of user data bytes
    // This means size in this case = 2 bytes bigger
    EXPECT_EQ((uint32_t)strlen(test_string[0]), stat.size);
}

TEST_F(FsTest, DeletedFileNoLongerExists)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_delete(fs, 0));
    EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
}


TEST_F(FsTest, CannotExceedMaxFilesOnFileSystem)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
    {
        EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, i, FS_MODE_CREATE, &wr_user_flags));
        EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
        EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
        EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    }
    EXPECT_EQ(FS_ERROR_FILESYSTEM_FULL, fs_open(fs, &handle, 65, FS_MODE_CREATE, &wr_user_flags));
}

TEST_F(FsTest, CannotCreateFileThatAlreadyExists)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_ERROR_FILE_ALREADY_EXISTS, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
}

TEST_F(FsTest, CannotExceedMaxFileHandles)
{
    fs_t fs;
    fs_handle_t handle[FS_PRIV_MAX_HANDLES + 1];
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < FS_PRIV_MAX_HANDLES; i++)
        EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle[i], i, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_ERROR_NO_FREE_HANDLE, fs_open(fs, &handle[FS_PRIV_MAX_HANDLES], FS_PRIV_MAX_HANDLES, FS_MODE_CREATE, &wr_user_flags));
}

TEST_F(FsTest, FileWriteAppend)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
            "Hello WorldHello World",
    };
    char buf[256];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[1]), rd);
    EXPECT_EQ(0, strncmp(test_string[1], buf, strlen(test_string[1])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, OpenNonExistentFileExpectFileNotFound)
{
    fs_t fs;
    fs_handle_t handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_open(fs, &handle, (uint8_t)i, FS_MODE_READONLY, NULL));
}

TEST_F(FsTest, DeleteNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_delete(fs, (uint8_t)i));
}

TEST_F(FsTest, StatNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 255; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_stat(fs, (uint8_t)i, NULL));
}

TEST_F(FsTest, ProtectNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_protect(fs, (uint8_t)i));
}

TEST_F(FsTest, UnprotectNonExistentFileExpectFileNotFound)
{
    fs_t fs;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    for (unsigned int i = 0; i < 256; i++)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_unprotect(fs, (uint8_t)i));
}

TEST_F(FsTest, StatEmptyFileSystemExpectMaxCapacityFree)
{
    fs_t fs;
    fs_stat_t stat;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(fs, FS_FILE_ID_NONE, &stat));
    EXPECT_EQ((uint32_t)FS_PRIV_USABLE_SIZE * FS_PRIV_MAX_SECTORS, stat.size);
}

TEST_F(FsTest, ProtectedFileCannotBeWritten)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][256] = {
            "Hello World",
    };

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_protect(fs, 0));
    EXPECT_EQ(FS_ERROR_FILE_PROTECTED, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));
}

TEST_F(FsTest, ProtectedFileCanBeRead)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
    };
    char buf[256];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_protect(fs, 0));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), rd);
    EXPECT_EQ(0, strncmp(test_string[0], buf, strlen(test_string[0])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, ToggledFileProtectionAllowsWrite)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
            "Hello WorldHello World"
    };
    char buf[256];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_protect(fs, 0));
    EXPECT_EQ(FS_NO_ERROR, fs_unprotect(fs, 0));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)strlen(test_string[1]), rd);
    EXPECT_EQ(0, strncmp(test_string[1], buf, strlen(test_string[1])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, FileCannotExceedFileSystemSize)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr;
    char test_string[][512] = {
            "DEADBEEFFEEDBEEF",
    };
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS * (FS_PRIV_PAGES_PER_SECTOR-1); i++)
    {
        EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], 510, &wr));
    }
    EXPECT_EQ(FS_ERROR_FILESYSTEM_FULL, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, FlushesNotLimitedIfNoDataWritten)
{
    fs_t fs;
    fs_handle_t handle;
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    trace_on = true;
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS * FS_PRIV_NUM_WRITE_SESSIONS; i++)
        EXPECT_EQ(FS_NO_ERROR, fs_flush(handle)); /* Should have no effect */
    EXPECT_EQ(FS_NO_ERROR, fs_flush(handle)); /* Should be accepted */
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, MultiFileIO)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][256] = {
            "Hello World",
            "Testing 1, 2, 3"
    };
    char buf[256];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], sizeof(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)sizeof(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 1, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[1], sizeof(test_string[1]), &wr));
    EXPECT_EQ((uint32_t)sizeof(test_string[1]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)sizeof(test_string[0]), rd);
    EXPECT_EQ(0, strncmp(test_string[0], buf, sizeof(test_string[0])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 1, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ((uint32_t)sizeof(test_string[1]), rd);
    EXPECT_EQ(0, strncmp(test_string[1], buf, sizeof(test_string[1])));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, FlashSectorWearLevellingIsApplied)
{
    fs_t fs;
    fs_handle_t handle;
    uint8_t wr_user_flags = 0x7;
    uint32_t wear_count[FS_PRIV_MAX_SECTORS];
    uint32_t min_wear_count;
    uint8_t min_sector;

    /* Pre-initialize flash with a random irregular flash wear profile */
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
    {
        wear_count[i] = rand() % 0xFFFFFFFF;
        SetSectorAllocCounter(i, wear_count[i]);
    }

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));

    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
    {
        EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, i, FS_MODE_CREATE, &wr_user_flags));
        EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
        /* Files should be allocated to sectors based on the minimum
         * wear level counter, so first find the minimum wear level counter
         * from the wear_count[] array.
         */
        min_wear_count = 0xFFFFFFFF;
        min_sector = 0xFF;
        for (unsigned int j = 0; j < FS_PRIV_MAX_SECTORS; j++)
        {
            if (wear_count[j] < min_wear_count)
            {
                min_wear_count = wear_count[j];
                min_sector = j;
            }
        }

        /* Check the file identifier was written to flash in
         * the correct sector based on wear levelling algorithm.
         */
        ASSERT_LT((uint8_t)min_sector, FS_PRIV_MAX_SECTORS);
        CheckFileId(min_sector, i);
        wear_count[min_sector] = 0xFFFFFFFF; /* Mark this sector as used */
    }
}

TEST_F(FsTest, StatEmptyFileShouldHaveZeroBytes)
{
    fs_t fs;
    fs_handle_t handle;
    fs_stat_t stat;
    uint8_t wr_user_flags = 0x7;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(fs, 0, &stat));
    EXPECT_EQ((uint32_t)0, stat.size);
}

TEST_F(FsTest, ReadEmptyFileShouldReturnEndOfFileError)
{
    fs_t fs;
    fs_handle_t handle;
    uint8_t wr_user_flags = 0x7;
    char buf[256];
    uint32_t rd;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_ERROR_END_OF_FILE, fs_read(handle, buf, sizeof(buf), &rd));
}

TEST_F(FsTest, LargeFileDataIntegrityCheck)
{
    fs_t fs;
    fs_handle_t handle;
    fs_stat_t stat;
    uint8_t wr_user_flags = 0x7;
    uint32_t wr, rd;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, &wr_user_flags));
    srand(0);
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS * (FS_PRIV_PAGES_PER_SECTOR - 1); i++)
    {
        for (unsigned int j = 0; j < FS_PRIV_PAGE_SIZE - 2; j++)
        {
            char x = (uint8_t)rand();
            EXPECT_EQ(FS_NO_ERROR, fs_write(handle, &x, 1, &wr));
            EXPECT_EQ((uint32_t)1, wr);
        }
    }
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    srand(0);
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS * (FS_PRIV_PAGES_PER_SECTOR - 1); i++)
    {
        for (unsigned int j = 0; j < FS_PRIV_PAGE_SIZE - 2; j++)
        {
            char x;
            EXPECT_EQ(FS_NO_ERROR, fs_read(handle, &x, 1, &rd));
            EXPECT_EQ((uint8_t)x, (uint8_t)rand());
            EXPECT_EQ((uint32_t)1, rd);
        }
    }
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));

    // Check file size is correct
    EXPECT_EQ(FS_NO_ERROR, fs_stat(fs, 0, &stat));
    EXPECT_EQ((uint32_t)FS_PRIV_MAX_SECTORS * (FS_PRIV_PAGES_PER_SECTOR-1) * (FS_PRIV_PAGE_SIZE - 2), stat.size);
}

TEST_F(FsTest, CircularFileCanOverwriteAndReadBack)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][512] = {
            "DEADBEEFFEEDBEEF", /* Even sector */
            "FEEDBEEFDEADBEEF", /* Odd sector */
    };
    char buf[512];
    uint8_t wr_user_flags = 0x7, rd_user_flags;

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE_CIRCULAR, &wr_user_flags));
    /* Fill file system with even/odd sector pattern */
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        for (unsigned int j = 0; j < FS_PRIV_PAGES_PER_SECTOR-1; j++)
            EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[i&1], FS_PRIV_PAGE_SIZE-2, &wr));
    /* Now overwrite previous root sector (even) which means new root sector should be odd */
    for (unsigned int i = 0; i < 1; i++)
        for (unsigned int j = 0; j < FS_PRIV_PAGES_PER_SECTOR-1; j++)
            EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[i&1], FS_PRIV_PAGE_SIZE-2, &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));

    /* Now open the file as read only */
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, &rd_user_flags));
    EXPECT_EQ(wr_user_flags, rd_user_flags);

    /* First sector should now be odd, 2nd even, etc */
    for (unsigned int i = 0; i < FS_PRIV_MAX_SECTORS; i++)
        for (unsigned int j = 0; j < FS_PRIV_PAGES_PER_SECTOR-1; j++)
        {
            EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, FS_PRIV_PAGE_SIZE-2, &rd));
            EXPECT_EQ(0, memcmp(test_string[(i+1) & 1], buf, FS_PRIV_PAGE_SIZE-2));
        }

    /* Next read should be EOF, even for circular file type */
    EXPECT_EQ(FS_ERROR_END_OF_FILE, fs_read(handle, buf, sizeof(buf), &rd));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, CorruptedFilesystem)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, rd;
    char test_string[][512] = {
            "Hello World",
    };
    uint8_t buf[512];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ((uint32_t)strlen(test_string[0]), wr);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));

    /* Forcibly corrupt the flash memory at the beginning of the first actual data page */
    flash_ram[FS_PRIV_PAGE_SIZE] = 0xee;
    flash_ram[FS_PRIV_PAGE_SIZE+1] = 0xee;

    /* Make sure this error is detected when trying to open the file */
    EXPECT_EQ(FS_ERROR_FILESYSTEM_CORRUPTED, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_ERROR_FILESYSTEM_CORRUPTED, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));

    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_flush(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_flush(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_flush(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], strlen(test_string[0]), &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_flush(handle));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));

    /* Corrupt again with a plausible header value which causes the headers to become out of sync */
    flash_ram[FS_PRIV_PAGE_SIZE] = 0x3;
    flash_ram[FS_PRIV_PAGE_SIZE+1] = 0x0;

    /* Make sure this error is detected when trying to open the file */
    EXPECT_EQ(FS_ERROR_FILESYSTEM_CORRUPTED, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_ERROR_FILESYSTEM_CORRUPTED, fs_open(fs, &handle, 0, FS_MODE_WRITEONLY, NULL));

    /* Now create a file that spans two sectors and corrupt only the 2nd sector.  This is
     * not detected during an open (since only 1st sector is scanned) but will be detected
     * during a read once the corrupted page is reached.
     */
    EXPECT_EQ(FS_NO_ERROR, fs_format(fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    for (unsigned int i = 0; i < 2; i++)
        for (unsigned int j = 0; j < FS_PRIV_PAGES_PER_SECTOR - 1; j++)
            EXPECT_EQ(FS_NO_ERROR, fs_write(handle, test_string[0], FS_PRIV_PAGE_SIZE-2, &wr));
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));

    /* Corrupt again with a plausible header value which causes the headers to become out of sync */
    flash_ram[(FS_PRIV_PAGES_PER_SECTOR * FS_PRIV_PAGE_SIZE) + 2 * FS_PRIV_PAGE_SIZE] = 0x3;
    flash_ram[(FS_PRIV_PAGES_PER_SECTOR * FS_PRIV_PAGE_SIZE) + 2 * FS_PRIV_PAGE_SIZE + 1] = 0x0;

    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_READONLY, NULL));

    /* Read first sector fine */
    for (unsigned int j = 0; j < FS_PRIV_PAGES_PER_SECTOR - 1; j++)
        EXPECT_EQ(FS_NO_ERROR, fs_read(handle, buf, FS_PRIV_PAGE_SIZE-2, &rd));

    /* Read 2nd sector will fail during end of sector scan */
    EXPECT_EQ(FS_ERROR_FILESYSTEM_CORRUPTED, fs_read(handle, buf, FS_PRIV_PAGE_SIZE-2, &rd));
    EXPECT_EQ((uint32_t)0, rd);
    EXPECT_EQ(FS_NO_ERROR, fs_close(handle));
}

TEST_F(FsTest, RandomWriteSize)
{
    fs_t fs;
    fs_handle_t handle;
    uint32_t wr, sz, total;
    uint8_t tx_buf[128];

    srand(0);
    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));

    total = 0;
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));

    while (true)
    {
        sz = rand() % sizeof(tx_buf);
        if (fs_write(handle, tx_buf, sz, &wr) != FS_NO_ERROR)
        {
            fs_close(handle);
            total = total + sz;
            break;
        }

        if ((rand() % 100) == 1)
            fs_flush(handle);

        total = total + sz;
    }
    EXPECT_GT(total, 16*1000000);
}

TEST_F(FsTest, ClobberHandleFileWriteDeathTest)
{
    fs_t fs;
    fs_handle_t handle;
    fs_priv_handle_t *priv;
    uint32_t wr;
    uint8_t tx_buf[16];

    EXPECT_EQ(FS_NO_ERROR, fs_init(0));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(0, &fs));
    EXPECT_EQ(FS_NO_ERROR, fs_open(fs, &handle, 0, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(handle, tx_buf, sizeof(tx_buf), &wr));
    priv = (fs_priv_handle_t *)handle;
    // This will corrupt the handle in a specific way which forces
    // a assert to trigger
    priv->last_data_offset = FS_PRIV_PAGE_SIZE - 4;
    EXPECT_DEATH(fs_write(handle, tx_buf, sizeof(tx_buf), &wr), ".*write_through_cache.*");
}
