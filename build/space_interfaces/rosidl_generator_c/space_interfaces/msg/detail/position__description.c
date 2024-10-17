// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from space_interfaces:msg/Position.idl
// generated code does not contain a copyright notice

#include "space_interfaces/msg/detail/position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_space_interfaces
const rosidl_type_hash_t *
space_interfaces__msg__Position__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xda, 0x98, 0xae, 0xe3, 0xb7, 0x8e, 0x2a, 0x19,
      0x33, 0x13, 0x21, 0x0a, 0xd3, 0x8f, 0xe6, 0x63,
      0xd0, 0x21, 0xcd, 0x9e, 0x2d, 0xae, 0x34, 0x16,
      0x01, 0x52, 0xf8, 0x69, 0x73, 0x8b, 0xc2, 0xc0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char space_interfaces__msg__Position__TYPE_NAME[] = "space_interfaces/msg/Position";

// Define type names, field names, and default values
static char space_interfaces__msg__Position__FIELD_NAME__source[] = "source";
static char space_interfaces__msg__Position__FIELD_NAME__target[] = "target";
static char space_interfaces__msg__Position__FIELD_NAME__x[] = "x";
static char space_interfaces__msg__Position__FIELD_NAME__y[] = "y";
static char space_interfaces__msg__Position__FIELD_NAME__z[] = "z";
static char space_interfaces__msg__Position__FIELD_NAME__roll[] = "roll";
static char space_interfaces__msg__Position__FIELD_NAME__pitch[] = "pitch";
static char space_interfaces__msg__Position__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field space_interfaces__msg__Position__FIELDS[] = {
  {
    {space_interfaces__msg__Position__FIELD_NAME__source, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {space_interfaces__msg__Position__FIELD_NAME__target, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {space_interfaces__msg__Position__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {space_interfaces__msg__Position__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {space_interfaces__msg__Position__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {space_interfaces__msg__Position__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {space_interfaces__msg__Position__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {space_interfaces__msg__Position__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
space_interfaces__msg__Position__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {space_interfaces__msg__Position__TYPE_NAME, 29, 29},
      {space_interfaces__msg__Position__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Position.msg\n"
  "\n"
  "# Represents the position and orientation of an object relative to a reference frame\n"
  "# Reference frames for the position and orientation\n"
  "# Source <--- Target\n"
  "\n"
  "string source \n"
  "string target \n"
  "\n"
  "# Position coordinates in 3D space\n"
  "\n"
  "float64 x \n"
  "float64 y \n"
  "float64 z \n"
  "\n"
  "# Orientation coordinates in 3D space\n"
  "\n"
  "float64 roll \n"
  "float64 pitch \n"
  "float64 yaw \n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
space_interfaces__msg__Position__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {space_interfaces__msg__Position__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 358, 358},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
space_interfaces__msg__Position__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *space_interfaces__msg__Position__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
