```
<BASE_DIR>/
└── <task_name>/
    └── ── meta/
        │   ├── info.json
        │   ├── tasks.jsonl
        |   |—— stats.json
        │   ├── episodes.jsonl
        │   └── episodes_stats.jsonl
        ├── data/
        │   └── chunk-000/
        │       └── episode_000000.parquet
        |       |—— episode_000001.parquet
        |       |—— ...
        ├── videos/
        │   └── chunk-000/
        │       └── observation.images.third_person/
        │           └── episode_000000.mp4
        |           |—— episode_000001.mp4
        │       └── observation.images.wrist/
        │           └── episode_000000.mp4
        |           |—— episode_000001.mp4
        
```