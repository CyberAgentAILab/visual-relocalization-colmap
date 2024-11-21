from huggingface_hub import snapshot_download

snapshot_download(repo_id="cyberagent/in-store-visual-localization",
                  repo_type="dataset",
                  local_dir="dataset")
