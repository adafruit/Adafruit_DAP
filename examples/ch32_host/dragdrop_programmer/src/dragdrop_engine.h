/*!
 * @file dragdrop_engine.h
 *
 * Drag-and-drop programming logic
 *
 * Interprets the sectors a host writes to the virtual drive.
 *
 */

#ifndef ADAFRUIT_DAP_CH32_DRAGDROP_ENGINE_H
#define ADAFRUIT_DAP_CH32_DRAGDROP_ENGINE_H

#include <stdint.h>

#include <Adafruit_DAP.h>
// ---- virtual FAT16 drive ----

#include <stdint.h>
#include <string.h>

/*!
 * @brief FAT12 volume whose sectors are generated when read.
 */
class VirtualFat {
public:
  static const uint32_t SECTOR_SIZE = 512;       //!< bytes per sector
  static const uint32_t TOTAL_SECTORS = 16384;   //!< 8 MB volume
  static const uint32_t SECTORS_PER_CLUSTER = 8; //!< 4 KB clusters
  static const uint32_t FAT_START = 1;           //!< first FAT sector
  static const uint32_t FAT_SECTORS = 6;         //!< sectors per FAT
  static const uint32_t ROOT_START = FAT_START + FAT_SECTORS; //!< root dir
  static const uint32_t ROOT_ENTRIES = 32; //!< root directory entries
  static const uint32_t ROOT_SECTORS =
      ROOT_ENTRIES * 32 / SECTOR_SIZE; //!< root directory sectors
  static const uint32_t DATA_START =
      ROOT_START + ROOT_SECTORS; //!< first data sector (cluster 2)
  static const uint32_t CLUSTERS =
      (TOTAL_SECTORS - DATA_START) / SECTORS_PER_CLUSTER; //!< data clusters
  static const uint8_t MAX_FILES = 4; //!< generated files at most

  /*!
   * @brief File content generator.
   * @param context Context given to addFile().
   * @param buf Receives the content.
   * @param max Buffer size (512).
   * @return Content length. Must not change until the drive is remounted.
   */
  typedef uint32_t (*Generator)(void *context, char *buf, uint32_t max);

  /*!
   * @brief Set the volume label.
   * @param label Up to 11 characters (upper-cased, padded with spaces).
   */
  void setVolumeLabel(const char *label);

  /*!
   * @brief Add a generated file (one cluster, at most 512 bytes).
   * @param name83 8.3 name as 11 characters without the dot, e.g.
   *        "DETAILS TXT".
   * @param gen Content generator.
   * @param context Passed to gen.
   * @return False if MAX_FILES files exist.
   */
  bool addFile(const char *name83, Generator gen, void *context);
  /*! @brief Remove all generated files. */
  void clearFiles(void) { _count = 0; }

  /*!
   * @brief Generate a sector.
   * @param lba Sector number.
   * @param buf Receives 512 bytes.
   */
  void readSector(uint32_t lba, uint8_t *buf);

  /*! @return First data sector after the generated files: where a host
   *          normally starts writing a newly copied file. */
  uint32_t firstFreeSector(void) const {
    return DATA_START + _count * SECTORS_PER_CLUSTER;
  }

private:
  void bootSector(uint8_t *buf);
  void fatSector(uint32_t index, uint8_t *buf);
  void rootSector(uint32_t index, uint8_t *buf);

  struct File {
    char name[11];
    Generator gen;
    void *context;
  };
  char _label[11] = {'D', 'A', 'P', 'L', 'I', 'N', 'K', ' ', ' ', ' ', ' '};
  File _files[MAX_FILES];
  uint8_t _count = 0;
};

// ---- drag-and-drop engine ----

/*!
 * @brief Drag-and-drop programming state machine.
 */
/*!
 * @brief Adafruit_DAP error handler for the drag-and-drop engine: records the
 *        error so a failed drop is reported on the drive (FAIL.TXT) instead
 *        of stopping the sketch. Pass it to Adafruit_DAP::begin().
 * @param text Error text.
 */
void dragdrop_dap_error(const char *text);

class DragDropEngine {
public:
  static const uint32_t QUIET_MS = 500;         //!< write gap ending a drop
  static const uint32_t REMOUNT_MS = 1500;      //!< "not ready" after a drop
  static const uint32_t UNRECOGNISED_MS = 1500; //!< wait for a session

  /*! Engine state. */
  enum State : uint8_t { IDLE, PROGRAMMING, REMOUNT };
  /*! Result of the last drop. */
  enum Result : uint8_t { NONE, OK, FAILED };

  void begin(Adafruit_DAP_STM32 *dap, const char *label);

  /*!
   * @brief Read a sector of the drive.
   * @param lba Sector number.
   * @param buf Receives 512 bytes.
   */
  void read(uint32_t lba, uint8_t *buf) { _fat.readSector(lba, buf); }
  bool write(uint32_t lba, const uint8_t *buf);
  void process(void);
  void poll(uint32_t now_ms);

  /*! @return True if write() handed over a sector for process(). */
  bool pending(void) const { return _pending; }
  /*! @return False while the drive should report "not ready". */
  bool unitReady(void) const { return _state != REMOUNT; }
  /*! @return Current state. */
  State state(void) const { return _state; }
  /*! @return Result of the last drop. */
  Result lastResult(void) const { return _result; }
  /*! @return Error text of the last failed drop. */
  const char *lastError(void) const { return _res_error; }
  /*! @return Bytes programmed by the last drop. */
  uint32_t lastBytes(void) const { return _res_bytes; }
  /*! @return True if a target was identified. */
  bool targetFound(void) const { return _found; }
  /*! @return Target name, or "none". */
  const char *targetName(void) const;
  /*! @return The virtual drive. */
  VirtualFat &fat(void) { return _fat; }

private:
  struct Seen {    // file entry from a root directory write
    uint32_t lba;  // first data sector
    uint32_t size; // bytes (0 until the host writes it)
    char name[11];
  };

  void rebuildFiles(void);
  void parseDir(const uint8_t *buf);
  const Seen *seenAt(uint32_t lba) const;
  static bool looksLikeImage(const uint8_t *buf);
  void fail(const char *error);
  void programSector(uint32_t lba, const uint8_t *buf);
  void finish(uint32_t now_ms, bool unrecognised_hex);
  bool connect(void);
  bool identify(void);
  bool select(void);
  bool programVerify(uint32_t addr, const uint8_t *buf, uint32_t len);
  static const char *nameFor(uint32_t id);
  static bool sectorAt(uint32_t offset, uint8_t *sector, uint32_t *start,
                       uint32_t *size);
  static uint32_t detailsText(void *context, char *buf, uint32_t max);
  static uint32_t failText(void *context, char *buf, uint32_t max);

  Adafruit_DAP_STM32 *_dap = nullptr;
  VirtualFat _fat;

  bool _found = false;
  uint32_t _dpidr = 0, _id = 0, _kb = 0; // target: DPIDR, DBGMCU_IDCODE, KB
  const char *_name = nullptr;

  State _state = IDLE;
  bool _pending = false, _selected = false, _failed = false, _held = false;
  const uint8_t *_p_buf = nullptr;
  uint32_t _p_lba = 0, _start_lba = 0, _erased = 0, _bytes = 0;
  uint32_t _t_start = 0, _t_last = 0, _t_dir = 0, _t_remount = 0;
  uint32_t _hold_lba = 0;
  const char *_error = nullptr;
  Seen _seen[4];
  uint8_t _seen_n = 0;
  uint8_t _hold[VirtualFat::SECTOR_SIZE];

  Result _result = NONE;
  const char *_res_error = nullptr;
  uint32_t _res_bytes = 0, _res_ms = 0;
  char _res_name[13] = {0};
};

#endif
