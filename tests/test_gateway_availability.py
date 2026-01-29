"""Test package availability checking with Gateway."""

import asyncio

from rattler import Channel, Gateway, Platform


def test_check_ros_jazzy_package():
    """Test checking for ros-jazzy-ros-base in robostack-jazzy channel."""
    channel_url = "https://prefix.dev/robostack-jazzy"
    package_name = "ros-jazzy-ros-base"
    platform = Platform.current()

    print(f"\nTesting package: {package_name}")
    print(f"Channel: {channel_url}")
    print(f"Platform: {platform}")

    gateway = Gateway()
    channel = Channel(channel_url)

    print(f"\nChannel created: {channel}")

    # Try querying with the gateway (it's async!)
    print("\nAttempting to query repo data...")
    repo_data_by_channel = asyncio.run(
        gateway.query(
            [channel],
            [platform],
            specs=[package_name],  # Use specs parameter, not package_names
            recursive=False,  # Don't fetch dependencies
        )
    )

    print(f"Repo data type: {type(repo_data_by_channel)}")
    print(f"Number of channels: {len(repo_data_by_channel)}")

    # repo_data_by_channel is a list of lists (one per channel)
    if repo_data_by_channel and repo_data_by_channel[0]:
        records = repo_data_by_channel[0]  # Get records from first channel
        print(f"Number of records in first channel: {len(records)}")
        for record in records:
            print(f"Found: {record.name} (normalized: {record.name.normalized}) - {record.version}")
        # Verify we found the package (compare normalized names)
        package_found = any(record.name.normalized == package_name.lower() for record in records)
        assert package_found, f"Package {package_name} not found"
        print(f"\n✓ Package {package_name} found successfully!")
    else:
        print("No records found")
        assert False, f"Package {package_name} not found in {channel_url}"


if __name__ == "__main__":
    test_check_ros_jazzy_package()
