use std::collections::HashMap;
use std::fs;
use std::path::Path;

pub type RawConfig = HashMap<String, String>;

/// Reads simple `keyword = value` lines. Lists and matrices come later.
pub fn load(path: &Path) -> Result<RawConfig, String> {
    let text = fs::read_to_string(path).map_err(|e| e.to_string())?;
    let mut config = RawConfig::new();

    for (line_no, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }

        let Some((key, value)) = line.split_once('=') else {
            return Err(format!("line {}: expected 'key = value'", line_no + 1));
        };

        config.insert(key.trim().to_string(), value.trim().to_string());
    }

    Ok(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_key_value_pairs() {
        let text = "method = direct\nname = test\n";
        let path = std::env::temp_dir().join("nurbs_reader_test.toml");
        std::fs::write(&path, text).unwrap();

        let config = load(&path).unwrap();
        assert_eq!(config.get("method"), Some(&"direct".to_string()));
        assert_eq!(config.get("name"), Some(&"test".to_string()));
    }
}
