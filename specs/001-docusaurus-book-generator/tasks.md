# Tasks: Docusaurus Book Generator

**Input**: Design documents from `specs/001-docusaurus-book-generator/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: User story task belongs to (e.g., US1)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure


- [X] T001 Create the top-level scripts directory in `scripts/`
- [X] T002 Create the book generator script file in `scripts/generate-book.mjs`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

- [X] T003 Implement command-line argument parsing in `scripts/generate-book.mjs` to accept an `--input` file path.
- [X] T004 Implement the logic to read and parse the JSON input configuration file in `scripts/generate-book.mjs`.

---

## Phase 3: User Story 1 - Generate Book Content (Priority: P1) 🎯 MVP

**Goal**: Automatically generate a Docusaurus book from a list of modules and chapters.

**Independent Test**: Provide a `book-definition.json` file and run the script. Verify that the corresponding markdown files are created in `docusaurus-book/docs/` with the correct content and structure.

### Implementation for User Story 1

- [X] T005 [US1] Implement the main loop in `scripts/generate-book.mjs` to iterate through the modules and chapters from the parsed configuration data.
- [X] T006 [US1] For each chapter, implement a function in `scripts/generate-book.mjs` that generates a prompt for the AI model to create content for a single chapter section (e.g., "Overview", "Key Concepts").
- [X] T007 [US1] Implement the logic in `scripts/generate-book.mjs` to call the Gemini API with the generated prompts to get the content for each section of a chapter.
- [X] T008 [US1] Implement a function in `scripts/generate-book.mjs` to assemble the full markdown content of a chapter from the AI-generated sections, ensuring they are in the correct order.
- [X] T009 [US1] Implement the file-writing logic in `scripts/generate-book.mjs` to save the complete chapter markdown to a new file within the `docusaurus-book/docs/` directory, using a kebab-case, numbered filename (e.g., `1-what-is-ros2.md`).

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect the feature's robustness and usability.

- [X] T010 Add robust error handling (e.g., for file not found, invalid JSON, API errors) in `scripts/generate-book.mjs`.
- [X] T011 Add logging (e.g., "Generating chapter X...", "Wrote file Y") to `scripts/generate-book.mjs` to provide feedback during generation.
- [X] T012 Create or update a `package.json` at the repository root to add a `generate-book` script that executes `node scripts/generate-book.mjs`.
- [X] T013 [P] Create a `README.md` in the `scripts/` directory explaining how to use the `generate-book.mjs` script, including the format of the input JSON file.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies.
- **Foundational (Phase 2)**: Depends on Setup completion.
- **User Story 1 (Phase 3)**: Depends on Foundational phase completion.
- **Polish (Phase 4)**: Depends on User Story 1 completion.

### Within User Story 1

- **T005** (Main Loop) is the entry point.
- **T006** (Prompt Generation) depends on T005.
- **T007** (API Call) depends on T006.
- **T008** (Content Assembly) depends on T007.
- **T009** (File Writing) depends on T008.

### Parallel Opportunities

- **T013** (README) can be done in parallel with most other tasks.
- Within the script's execution, the generation of content for different chapters (T007) could potentially be parallelized with asynchronous API calls.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational.
3.  Complete Phase 3: User Story 1.
4.  **STOP and VALIDATE**: Test the script with a sample `book-definition.json` and ensure it generates the correct markdown files.

### Incremental Delivery

The feature is a single user story, so incremental delivery applies to the tasks within the story. A good approach would be to first implement the script for a single, hardcoded chapter, then generalize it to handle the full configuration file.
