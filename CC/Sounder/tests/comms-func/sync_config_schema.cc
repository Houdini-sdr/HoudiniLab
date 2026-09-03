/**
 * @file sync_config_schema.cc
 * @brief Print the sync knob table as markdown, from the one table in
 *        SyncConfig, for the walkthrough. Run: ./sync_config_schema
 */
#include <cstdio>
#include <string>

#include "sync/sync_config.h"

int main() {
  const std::string m = houdini::sync::SyncConfig::schemaMarkdown();
  std::fwrite(m.data(), 1, m.size(), stdout);
  return 0;
}
