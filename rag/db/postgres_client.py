import os
import psycopg2
import psycopg2.extras
from typing import List, Dict, Optional
import json
from datetime import datetime
import asyncio


class PostgresConfig:
    def __init__(self):
        self.database_url = os.getenv("NEON_DATABASE_URL")
        self.connection = None

    def connect(self):
        """Establish connection to the database"""
        try:
            self.connection = psycopg2.connect(self.database_url)
            self._create_tables()
        except Exception as e:
            print(f"Error connecting to PostgreSQL: {e}")
            raise

    def _create_tables(self):
        """Create required tables if they don't exist"""
        with self.connection.cursor() as cursor:
            # Create tables for user queries and chatbot responses
            create_queries_table = """
            CREATE TABLE IF NOT EXISTS user_queries (
                id TEXT PRIMARY KEY,
                user_id TEXT,
                query_text TEXT NOT NULL,
                selected_text TEXT,
                context_page TEXT,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                session_id TEXT NOT NULL
            );
            """

            create_responses_table = """
            CREATE TABLE IF NOT EXISTS chatbot_responses (
                id TEXT PRIMARY KEY,
                query_id TEXT NOT NULL,
                response_text TEXT NOT NULL,
                citations JSONB,
                follow_up_questions JSONB,
                confidence_score REAL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                model_used TEXT NOT NULL,
                FOREIGN KEY (query_id) REFERENCES user_queries(id)
            );
            """

            create_content_metadata_table = """
            CREATE TABLE IF NOT EXISTS content_metadata (
                content_id TEXT PRIMARY KEY,
                version TEXT NOT NULL,
                authors TEXT[],
                reviewers TEXT[],
                status TEXT NOT NULL,
                language TEXT DEFAULT 'en',
                estimated_reading_time INTEGER,
                related_content_ids TEXT[],
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            """

            cursor.execute(create_queries_table)
            cursor.execute(create_responses_table)
            cursor.execute(create_content_metadata_table)
            self.connection.commit()

    def store_user_query(self, query_data: Dict) -> bool:
        """Store user query in the database"""
        try:
            with self.connection.cursor() as cursor:
                query = """
                INSERT INTO user_queries (id, user_id, query_text, selected_text, context_page, timestamp, session_id)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                """
                cursor.execute(
                    query,
                    (
                        query_data['id'],
                        query_data.get('user_id'),
                        query_data['query_text'],
                        query_data.get('selected_text'),
                        query_data.get('context_page'),
                        query_data['timestamp'],
                        query_data['session_id']
                    )
                )
                self.connection.commit()
                return True
        except Exception as e:
            print(f"Error storing user query: {e}")
            return False

    def store_chatbot_response(self, response_data: Dict) -> bool:
        """Store chatbot response in the database"""
        try:
            with self.connection.cursor() as cursor:
                query = """
                INSERT INTO chatbot_responses (id, query_id, response_text, citations, follow_up_questions,
                                             confidence_score, timestamp, model_used)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                """
                cursor.execute(
                    query,
                    (
                        response_data['id'],
                        response_data['query_id'],
                        response_data['response_text'],
                        json.dumps(response_data.get('citations', [])),
                        json.dumps(response_data.get('follow_up_questions', [])),
                        response_data.get('confidence_score'),
                        response_data['timestamp'],
                        response_data['model_used']
                    )
                )
                self.connection.commit()
                return True
        except Exception as e:
            print(f"Error storing chatbot response: {e}")
            return False

    def get_chat_history(self, session_id: str) -> List[Dict]:
        """Retrieve chat history for a session"""
        try:
            with self.connection.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cursor:
                query = """
                SELECT uq.query_text, uq.timestamp as query_timestamp,
                       cr.response_text, cr.timestamp as response_timestamp
                FROM user_queries uq
                LEFT JOIN chatbot_responses cr ON uq.id = cr.query_id
                WHERE uq.session_id = %s
                ORDER BY uq.timestamp
                """
                cursor.execute(query, (session_id,))
                rows = cursor.fetchall()

                history = []
                for row in rows:
                    history.append({
                        'query': row['query_text'],
                        'query_timestamp': row['query_timestamp'],
                        'response': row['response_text'],
                        'response_timestamp': row['response_timestamp']
                    })

                return history
        except Exception as e:
            print(f"Error retrieving chat history: {e}")
            return []

    def store_content_metadata(self, metadata: Dict) -> bool:
        """Store content metadata in the database"""
        try:
            with self.connection.cursor() as cursor:
                query = """
                INSERT INTO content_metadata (content_id, version, authors, reviewers, status,
                                            language, estimated_reading_time, related_content_ids,
                                            created_at, updated_at)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                ON CONFLICT (content_id) DO UPDATE SET
                    version = EXCLUDED.version,
                    authors = EXCLUDED.authors,
                    reviewers = EXCLUDED.reviewers,
                    status = EXCLUDED.status,
                    language = EXCLUDED.language,
                    estimated_reading_time = EXCLUDED.estimated_reading_time,
                    related_content_ids = EXCLUDED.related_content_ids,
                    updated_at = CURRENT_TIMESTAMP
                """
                cursor.execute(
                    query,
                    (
                        metadata['content_id'],
                        metadata['version'],
                        metadata.get('authors', []),
                        metadata.get('reviewers', []),
                        metadata['status'],
                        metadata.get('language', 'en'),
                        metadata.get('estimated_reading_time'),
                        metadata.get('related_content_ids', []),
                        metadata.get('created_at', datetime.utcnow()),
                        metadata.get('updated_at', datetime.utcnow())
                    )
                )
                self.connection.commit()
                return True
        except Exception as e:
            print(f"Error storing content metadata: {e}")
            return False

    def get_content_metadata(self, content_id: str) -> Optional[Dict]:
        """Retrieve content metadata by content ID"""
        try:
            with self.connection.cursor(cursor_factory=psycopg2.extras.RealDictCursor) as cursor:
                query = "SELECT * FROM content_metadata WHERE content_id = %s"
                cursor.execute(query, (content_id,))
                row = cursor.fetchone()
                if row:
                    return dict(row)
                return None
        except Exception as e:
            print(f"Error retrieving content metadata: {e}")
            return None

# Global instance
postgres_config = PostgresConfig()