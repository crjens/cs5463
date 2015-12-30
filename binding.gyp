{
    "targets": [
        {
            "target_name": "cs5463",
            "include_dirs": [ "<!(node -e \"require('nan')\")" ],
            "sources": [ "CS5463.cc"],
            "link_settings": { "libraries": [ "-lwiringPi" ] }
        }
    ]
}