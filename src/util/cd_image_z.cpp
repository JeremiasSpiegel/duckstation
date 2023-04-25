// SPDX-FileCopyrightText: 2019-2022 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#include "cd_image.h"
#include "cd_subchannel_replacement.h"
#include "common/error.h"
#include "common/file_system.h"
#include "common/log.h"
#include <cerrno>
#include <cstdlib>
#include <zlib.h>
#include <bzlib.h>
Log_SetChannel(CDImageZ);

class CDImageZ : public CDImage
{
public:
  CDImageZ();
  ~CDImageZ() override;

  bool Open(const char* filename, Common::Error* error);

  bool ReadSubChannelQ(SubChannelQ* subq, const Index& index, LBA lba_in_index) override;
  bool HasNonStandardSubchannel() const override;

protected:
  bool ReadSectorFromIndex(void* buffer, const Index& index, LBA lba_in_index) override;

private:
  std::FILE* m_fp = nullptr;
  std::FILE* m_fp_table = nullptr;
  u64 m_file_position = 0;
  u16 m_compressed_blocksize;
  u16 m_num_compressed_blocks;
  s32 m_compressed_block_start_index;

  CDSubChannelReplacement m_sbi;
  bool isBZ2;

  uint8_t m_tmp_decompressed[3000 * 10];  // TODO
  uint8_t m_tmp_buffer[3000 * 10];      // TODO
};

CDImageZ::CDImageZ() = default;

CDImageZ::~CDImageZ()
{
  if (m_fp)
    std::fclose(m_fp);
}

bool CDImageZ::Open(const char* filename, Common::Error* error)
{
  m_filename = filename;

  std::string bzsuffix = ".bz";
  isBZ2 = (m_filename.rfind(bzsuffix) == m_filename.size() - bzsuffix.size());

  std::string filename_table = m_filename + ".table";
  m_fp = FileSystem::OpenCFile(filename, "rb");
  if (!m_fp)
  {
    Log_ErrorPrintf("Failed to open z file '%s': errno %d", filename, errno);
    if (error)
      error->SetErrno(errno);
    return false;
  }

  m_fp_table = FileSystem::OpenCFile(filename_table.c_str(), "rb");
  if (!m_fp_table)
  {
    Log_ErrorPrintf("Failed to open z table file '%s': errno %d", filename_table.c_str(), errno);
    if (error)
      error->SetErrno(errno);
    return false;
  }

  if (isBZ2) {
      m_compressed_blocksize = RAW_SECTOR_SIZE * 10;
      m_num_compressed_blocks = 10;
      m_compressed_block_start_index = -1;
  } else {
      m_compressed_blocksize = RAW_SECTOR_SIZE;
      m_num_compressed_blocks = 1;
  }

  Log_InfoPrintf("Z and index file open");

  const u32 track_sector_size = RAW_SECTOR_SIZE;

  // determine the length from the file
  std::fseek(m_fp_table, 0, SEEK_END);
  const u32 file_size = static_cast<u32>(std::ftell(m_fp_table) / 6 * track_sector_size * m_num_compressed_blocks) ;
  std::fseek(m_fp_table, 0, SEEK_SET);

  Log_InfoPrintf("Calculated image size %i", file_size);

  m_lba_count = file_size / track_sector_size;

  SubChannelQ::Control control = {};
  TrackMode mode = TrackMode::Mode2Raw;
  control.data = mode != TrackMode::Audio;

  // Two seconds default pregap.
  const u32 pregap_frames = 2 * FRAMES_PER_SECOND;
  Index pregap_index = {};
  pregap_index.file_sector_size = track_sector_size;
  pregap_index.start_lba_on_disc = 0;
  pregap_index.start_lba_in_track = static_cast<LBA>(-static_cast<s32>(pregap_frames));
  pregap_index.length = pregap_frames;
  pregap_index.track_number = 1;
  pregap_index.index_number = 0;
  pregap_index.mode = mode;
  pregap_index.control.bits = control.bits;
  pregap_index.is_pregap = true;
  m_indices.push_back(pregap_index);

  // Data index.
  Index data_index = {};
  data_index.file_index = 0;
  data_index.file_offset = 0;
  data_index.file_sector_size = track_sector_size;
  data_index.start_lba_on_disc = pregap_index.length;
  data_index.track_number = 1;
  data_index.index_number = 1;
  data_index.start_lba_in_track = 0;
  data_index.length = m_lba_count;
  data_index.mode = mode;
  data_index.control.bits = control.bits;
  m_indices.push_back(data_index);

  // Assume a single track.
  m_tracks.push_back(
    Track{static_cast<u32>(1), data_index.start_lba_on_disc, static_cast<u32>(0), m_lba_count, mode, control});

  AddLeadOutIndex();

  m_sbi.LoadSBIFromImagePath(filename);

  return Seek(1, Position{0, 0, 0});
}

bool CDImageZ::ReadSubChannelQ(SubChannelQ* subq, const Index& index, LBA lba_in_index)
{
  if (m_sbi.GetReplacementSubChannelQ(index.start_lba_on_disc + lba_in_index, subq))
    return true;

  return CDImage::ReadSubChannelQ(subq, index, lba_in_index);
}

bool CDImageZ::HasNonStandardSubchannel() const
{
  return (m_sbi.GetReplacementSectorCount() > 0);
}

bool CDImageZ::ReadSectorFromIndex(void* buffer, const Index& index, LBA lba_in_index)
{
  const u64 file_position = index.file_offset + (static_cast<u64>(lba_in_index) * index.file_sector_size);

  u64 lba = file_position / index.file_sector_size;

  if (isBZ2)
  {
      // Check if the block is already in cache
      if ((lba > m_compressed_block_start_index ) && (lba <= m_compressed_block_start_index + m_num_compressed_blocks))
      {
          u64 idx_in_block = lba - m_compressed_block_start_index;
          std::memcpy(buffer, m_tmp_decompressed + idx_in_block * index.file_sector_size, index.file_sector_size);
      }
  }

  Log_DebugPrintf("Reading lba: (%lu)", lba);

  long lba_offset_in_table = static_cast<long>(lba / m_num_compressed_blocks * 6);

  Log_DebugPrintf("lba offset: (%lu - %lu)", lba_offset_in_table, lba_offset_in_table / 6);

  if (std::fseek(m_fp_table, lba_offset_in_table, SEEK_SET) != 0)
  {
      Log_ErrorPrintf("Could not seek in table file: errno %d", errno);
      return false;
  }
  uint32_t offset;
  uint16_t length;
  if (std::fread(&offset, sizeof(uint32_t), 1, m_fp_table) != 1)
  {
      Log_ErrorPrintf("Could not read offset from table file: errno %d", errno);
      return false;
  }
  Log_DebugPrintf("bin offset: (%u)", offset);

  if (std::fread(&length, sizeof(uint16_t), 1, m_fp_table) != 1)
  {
      Log_ErrorPrintf("Could not read length from table file: errno %d", errno);
      return false;
  }
  Log_DebugPrintf("bin length: (%u)", length);

  if (std::fseek(m_fp, static_cast<long>(offset), SEEK_SET) != 0)
  {
      Log_ErrorPrintf("Could not seek in compressed file: errno %d", errno);
      return false;
  }
  if (std::fread(m_tmp_buffer, length, 1, m_fp) != 1)
  {
    Log_ErrorPrintf("Could not read from compressed file: errno %d", errno);
    std::fseek(m_fp_table, static_cast<long>(m_file_position), SEEK_SET);
    return false;
  }
  else
  {
    if (!isBZ2)
    {
        unsigned long out_size = index.file_sector_size;
        // Normal zlib compression
        int res = uncompress(static_cast<unsigned char *>(buffer), &out_size, m_tmp_buffer, length);
        if (res != Z_OK) {
            Log_ErrorPrintf("Decompression error.\n");
            return false;
        }
    }
    else
    {
        unsigned int out_size = index.file_sector_size * m_num_compressed_blocks;
        int res = BZ2_bzBuffToBuffDecompress(reinterpret_cast<char *>(m_tmp_decompressed), &out_size, reinterpret_cast<char *>(m_tmp_buffer), length, 0, 0);
        if (res != BZ_OK) {
            Log_ErrorPrintf("Decompression error.\n");
            return false;
        }
        m_compressed_block_start_index = static_cast<long>(lba / m_num_compressed_blocks) * m_num_compressed_blocks;
        u64 idx_in_block = lba - m_compressed_block_start_index;
        std::memcpy(buffer, m_tmp_decompressed + idx_in_block * index.file_sector_size, index.file_sector_size);
    }
  }

  m_file_position += index.file_sector_size;
  return true;
}

std::unique_ptr<CDImage> CDImage::OpenZCompressedImage(const char* filename, Common::Error* error)
{
  std::unique_ptr<CDImageZ> image = std::make_unique<CDImageZ>();
  if (!image->Open(filename, error))
    return {};

  return image;
}
